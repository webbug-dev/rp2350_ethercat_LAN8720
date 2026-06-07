#include "shared_state.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware/timer.h"
#include <string.h>

shared_state_t    g_shared;
SemaphoreHandle_t g_shared_mutex;

void shared_state_init(void) {
    memset(&g_shared, 0, sizeof(g_shared));
    for (int i = 0; i < EC_DRIVE_COUNT; i++) {
        g_shared.drive[i].phase = DS_SCAN;
        strcpy(g_shared.drive[i].msg, "Scanning...");
    }
    g_shared_mutex = xSemaphoreCreateMutex();
    configASSERT(g_shared_mutex);
}

// FreeRTOS run-time stats use the pico microsecond hardware counter.
void     stats_timer_init(void) { /* hardware timer already running */ }
uint32_t stats_timer_get(void)  { return (uint32_t)time_us_32(); }
