#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"

// 1.5" SH1107 128x128 monochrome OLED over I2C — instance-based so several
// panels (each on its own I2C bus) can be driven independently.

#define SH1107_W      128
#define SH1107_H      128
#define SH1107_PAGES  (SH1107_H / 8)          // 16 pages of 8 rows

// One panel. Fill in the config fields, then call sh1107_init().
typedef struct {
    i2c_inst_t *i2c;        // i2c0 / i2c1
    uint        sda_pin;
    uint        scl_pin;
    uint        i2c_hz;     // e.g. 400000
    uint8_t     addr;       // 0x3C / 0x3D
    int         rotation;   // 0 / 90 / 180 / 270 (degrees CW)
    int         x_shift;    // cyclic logical shift (wraps), cancels panel offset
    int         y_shift;
    const char *name;       // for logging ("OLED1", "OLED2")
    // Optional bus mutex: set when this panel shares its I2C bus with another
    // device (OLED1 shares I2C0 with DAC1). NULL = sole bus owner, no locking.
    SemaphoreHandle_t bus_mutex;
    // ── private ──
    uint8_t     fb[SH1107_PAGES * SH1107_W];  // page-major framebuffer
} sh1107_t;

void sh1107_init(sh1107_t *d);
void sh1107_clear(sh1107_t *d);                          // clear framebuffer (RAM)
void sh1107_set_pixel(sh1107_t *d, int x, int y, bool on);   // logical coords
void sh1107_fill_rect(sh1107_t *d, int x, int y, int w, int h, bool on);
void sh1107_show(sh1107_t *d);                           // push whole FB to the panel
// Push only the pages covered by a LOGICAL rectangle — much faster than a full
// flush for small UI updates (keeps the CPU/I2C bus free for button polling).
void sh1107_show_area(sh1107_t *d, int lx1, int ly1, int lx2, int ly2);

// Calibration aid: outer/inner frames + per-corner dot markers (1 dot in the
// top-left corner, 2 top-right, 3 bottom-left, 4 bottom-right) + a diagonal.
// Lets you read off rotation/offset from what's on the glass.
void sh1107_test_pattern(sh1107_t *d);
