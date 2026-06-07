#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

#include "FreeRTOS.h"
#include "task.h"

// Crash-reason carried across a watchdog reset in scratch[0] (survives reset,
// not power-on). Lets the next boot report why the previous run died even though
// USB was frozen at the time. See the fault hooks at the bottom of this file.
#define CRASH_MAGIC      0xC0DE0000u
#define CRASH_HARDFAULT  1u
#define CRASH_STACKOVF   2u
#define CRASH_MALLOC     3u

#include "board.h"
#include "shared_state.h"
#include "ethercat.h"
#include "display_task.h"
#include "periph.h"
#include "log.h"

static void boot_task(void *arg) {
    (void)arg;
    // Bring up the async logger FIRST, before any LOG() line: queue + drain task
    // pinned to core 0 (alongside the UI/buttons, and the USB-CDC console).
    // Lowest app priority so draining never preempts real work — bursts buffer
    // in the queue, overflow is dropped.
    log_init(tskIDLE_PRIORITY + 2, (1u << 0));

    shared_state_init();

    // Bring up the board peripherals we still use: the two OLED I2C buses and
    // the two run buttons (GP2 = Servo, GP3 = Step). Cheap, non-blocking.
    periph_init();

    // Bring the OLED up FIRST: the user sees the "Booting…" screen (with the
    // menu-bar loader animating) within a fraction of a second, instead of a
    // blank panel while we wait for USB and the network to come up.
    display_task_start(tskIDLE_PRIORITY + 3);   // core 1 — LVGL UI

    // Hardware watchdog: the display task (core 0) feeds it every loop. A full
    // dual-core lockup (which freezes USB and defeats the fault handlers + soft
    // reset) stops the feeding, so the chip auto-resets in ~6 s and stays
    // recoverable. Core-1-only hangs leave core 0 + USB alive (still soft-
    // resettable), so the generous timeout won't false-fire on long EtherCAT scan calls.
    watchdog_enable(6000, 1);

    // Now give a USB-CDC host a short window to attach so the early boot log is
    // captured. The display is already alive and animating throughout this —
    // it no longer blocks on USB (this used to be an 8s pre-scheduler stall).
    for (int i = 0; i < 20 && !stdio_usb_connected(); i++)
        vTaskDelay(pdMS_TO_TICKS(100));

    LOG("\r\n\r\n=== RP2350 + LAN8720 (RMII) EtherCAT master + SH1107 (LVGL) ===\r\n");
    LOG("pico-sdk + FreeRTOS SMP + LVGL\r\n\r\n");
    LOG("[BOOT] clk_sys=%lu Hz, FreeRTOS heap=%u bytes\n",
        (unsigned long)clock_get_hz(clk_sys), (unsigned)configTOTAL_HEAP_SIZE);

    // Report (and clear) the reason the previous run died, if it was a fault
    // that auto-rebooted us via the watchdog.
    uint32_t cr = watchdog_hw->scratch[0];
    watchdog_hw->scratch[0] = 0;
    if ((cr & 0xFFFF0000u) == CRASH_MAGIC) {
        const char *r = (cr & 0xFFFFu) == CRASH_HARDFAULT ? "HARD FAULT" :
                        (cr & 0xFFFFu) == CRASH_STACKOVF  ? "STACK OVERFLOW" :
                        (cr & 0xFFFFu) == CRASH_MALLOC    ? "MALLOC FAILED" : "?";
        LOG("[BOOT] *** previous run crashed: %s — auto-rebooted ***\n", r);
        if ((cr & 0xFFFFu) == CRASH_HARDFAULT)
            LOG("[BOOT]     fault PC = 0x%08lX\n", (unsigned long)watchdog_hw->scratch[1]);
    } else if (watchdog_caused_reboot()) {
        LOG("[BOOT] *** previous run LOCKED UP (hardware watchdog reset) ***\n");
    }

    ethercat_start(tskIDLE_PRIORITY + 4);       // core 0 — EtherCAT fieldbus

    LOG("[BOOT] tasks launched\n");
    vTaskDelete(NULL);
}

int main(void) {
    // The PIO RMII MAC re-syncs to REF_CLK every di-bit, so the system clock just
    // needs to be fast enough to catch the 50 MHz edges: SYS_CLOCK_KHZ (250 MHz).
    // Bump the core voltage first (optional, see CORE_VREG_BOOST in board.h),
    // then raise the clock — both before USB/stdio so clk_peri is settled.
#if CORE_VREG_BOOST
    vreg_set_voltage(CORE_VREG_VOLTAGE);
#endif
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    stdio_init_all();

    TaskHandle_t h = NULL;
    BaseType_t r = xTaskCreate(boot_task, "boot", 1024, NULL,
                               tskIDLE_PRIORITY + 5, &h);
    configASSERT(r == pdPASS);
    vTaskCoreAffinitySet(h, (UBaseType_t)(1u << 0));

    vTaskStartScheduler();
    for (;;) {}
}

// Fault recovery: instead of hanging (which freezes USB and needs a physical
// power-cycle), record the cause and reboot via the watchdog. The next boot
// re-enumerates USB and prints the reason, so a crash is observable + the board
// stays flashable.
void vApplicationMallocFailedHook(void) {
    watchdog_hw->scratch[0] = CRASH_MAGIC | CRASH_MALLOC;
    watchdog_reboot(0, 0, 0);
    for (;;) {}
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
    (void)task; (void)name;
    watchdog_hw->scratch[0] = CRASH_MAGIC | CRASH_STACKOVF;
    watchdog_reboot(0, 0, 0);
    for (;;) {}
}

// Override the SDK's hard-fault handler: capture the faulting PC (from the
// stacked exception frame) into scratch[1], record the cause, and reboot. The
// next boot prints the PC so it can be mapped to a function with addr2line.
static volatile uint32_t g_fault_pc;
void hardfault_finish(void);

void __attribute__((naked)) isr_hardfault(void) {
    __asm volatile(
        "tst lr, #4            \n"   // EXC_RETURN bit2: which stack was used
        "ite eq                \n"
        "mrseq r0, msp         \n"
        "mrsne r0, psp         \n"
        "ldr r1, [r0, #24]     \n"   // stacked PC = frame[6]
        "ldr r2, =g_fault_pc   \n"
        "str r1, [r2]          \n"
        "b hardfault_finish    \n"
    );
}

void hardfault_finish(void) {
    watchdog_hw->scratch[1] = g_fault_pc;
    watchdog_hw->scratch[0] = CRASH_MAGIC | CRASH_HARDFAULT;
    watchdog_reboot(0, 0, 0);
    for (;;) {}
}
