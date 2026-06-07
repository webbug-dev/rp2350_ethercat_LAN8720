#pragma once
#include <stdint.h>
#include <stdbool.h>

// periph.* — the on-board peripherals this firmware still uses:
//   • the I2C bus that carries the SH1107 OLED panel
//   • two active-low push buttons (GP2 = Servo run, GP3 = Step run)
//
// periph_init() brings both up at boot.

#include "FreeRTOS.h"
#include "semphr.h"

void periph_init(void);

// ── I2C bus (OLED on I2C0) ──────────────────────────────────────────────────
void i2c_buses_init(void);
// Bus mutex, held across every OLED transaction.
extern SemaphoreHandle_t g_oled_mutex;

// ── Buttons (active-low, internal pull-up; press = short to GND) ─────────────
void buttons_init(void);
// True while the button is physically pressed (pin reads 0). idx: 0..BTN_COUNT-1.
bool button_down(int idx);
