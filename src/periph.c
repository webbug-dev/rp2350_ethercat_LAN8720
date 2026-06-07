#include "periph.h"
#include "board.h"
#include "log.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "FreeRTOS.h"
#include "semphr.h"

// ── I2C bus ─────────────────────────────────────────────────────────────────
SemaphoreHandle_t g_oled_mutex;

void i2c_buses_init(void) {
    i2c_init(OLED_I2C_INST, OLED_I2C_HZ);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN);
    gpio_pull_up(OLED_SCL_PIN);
    g_oled_mutex = xSemaphoreCreateMutex();
    configASSERT(g_oled_mutex);

    LOG("[PERIPH] I2C0 SDA=GP%d SCL=GP%d (OLED) @ %dkHz\n",
        OLED_SDA_PIN, OLED_SCL_PIN, OLED_I2C_HZ / 1000);
}

// ── Buttons ─────────────────────────────────────────────────────────────────
static const uint s_btn_pin[BTN_COUNT] = { BTN0_PIN, BTN1_PIN };

void buttons_init(void) {
    for (int i = 0; i < BTN_COUNT; i++) {
        gpio_init(s_btn_pin[i]);
        gpio_set_dir(s_btn_pin[i], GPIO_IN);
        gpio_pull_up(s_btn_pin[i]);          // active-low → idle high
    }
    LOG("[PERIPH] buttons: GP%d (Servo run), GP%d (Step run) — active-low\n",
        BTN0_PIN, BTN1_PIN);
}

bool button_down(int idx) {
    if (idx < 0 || idx >= BTN_COUNT) return false;
    return gpio_get(s_btn_pin[idx]) == 0;    // active-low (pressed = to GND)
}

// ── Aggregate ───────────────────────────────────────────────────────────────
void periph_init(void) {
    i2c_buses_init();
    buttons_init();
}
