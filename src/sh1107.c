#include "sh1107.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "log.h"
#include <string.h>

// Orientation knobs — flip these if the image is mirrored/upside-down on
// your particular GME128128 panel:
//   segment remap : 0xA0 (normal) / 0xA1 (flipped)
//   common scan   : 0xC0 (normal) / 0xC8 (flipped)
#define SH1107_SEGREMAP    0xA1
#define SH1107_COMSCAN     0xC8
#define SH1107_DISP_OFFSET 0x60      // 128x128 1.5" panels use 0x60

// Bus serialization — only takes effect when a panel shares its I2C bus with
// another device (OLED1 ↔ DAC1 on I2C0). NULL mutex = no-op.
static void bus_lock(sh1107_t *d)   { if (d->bus_mutex) xSemaphoreTake(d->bus_mutex, portMAX_DELAY); }
static void bus_unlock(sh1107_t *d) { if (d->bus_mutex) xSemaphoreGive(d->bus_mutex); }

// Timeout-based I2C so a missing / miswired OLED can never block the display
// task forever (a blocking write to an unresponsive bus would hang). Does not
// take the bus mutex — callers hold it across the whole operation.
static void cmd(sh1107_t *d, uint8_t c) {
    uint8_t b[2] = { 0x00, c };      // Co=0, D/C#=0 → command stream
    i2c_write_timeout_us(d->i2c, d->addr, b, 2, false, 3000);
}

// Push the framebuffer to the panel (caller holds the bus mutex).
static void flush_fb(sh1107_t *d) {
    uint8_t line[1 + SH1107_W];
    line[0] = 0x40;                              // data stream
    for (int page = 0; page < SH1107_PAGES; page++) {
        cmd(d, 0xB0 | page);                     // set page address
        cmd(d, 0x00);                            // lower column = 0
        cmd(d, 0x10);                            // higher column = 0
        memcpy(&line[1], &d->fb[page * SH1107_W], SH1107_W);
        i2c_write_timeout_us(d->i2c, d->addr, line, sizeof(line), false, 20000);
    }
}

void sh1107_init(sh1107_t *d) {
    LOG("[OLED] %s SH1107 init: SDA=%u SCL=%u addr=0x%02X %dx%d%s\n",
        d->name ? d->name : "?", d->sda_pin, d->scl_pin, d->addr,
        SH1107_W, SH1107_H, d->bus_mutex ? " (shared bus)" : "");
    // The I2C bus itself (pins, clock, pull-ups) is brought up by the board
    // layer (periph i2c_buses_init) since OLED1 shares I2C0 with DAC1.
    sleep_ms(50);

    bus_lock(d);
    // Battle-tested Adafruit SH110X 128x128 init sequence.
    cmd(d, 0xAE);                    // display off
    cmd(d, 0xD5); cmd(d, 0xF0);      // max osc freq + min divide → high frame rate
                                     // (reduces camera flicker from the row scan)
    cmd(d, 0x81); cmd(d, 0xFF);      // contrast = max (this is the safe "brightest")
    cmd(d, 0xAD); cmd(d, 0x8A);      // DC-DC control (built-in pump on)
    cmd(d, SH1107_SEGREMAP);         // segment remap
    cmd(d, SH1107_COMSCAN);          // common output scan direction
    cmd(d, 0xDC); cmd(d, 0x00);      // display start line = 0
    cmd(d, 0xD3); cmd(d, SH1107_DISP_OFFSET); // display offset
    // Pre-charge / VCOMH kept at the datasheet-nominal values. Pushing pre-charge
    // to max + VCOMH to min ("overdrive") draws more from the on-chip charge pump
    // than it can hold steady → audible buzz and a visible flicker, so we don't.
    cmd(d, 0xD9); cmd(d, 0x11);      // short pre-charge → a bit more frame rate
    cmd(d, 0xDB); cmd(d, 0x35);      // VCOM deselect level (nominal)
    cmd(d, 0xA8); cmd(d, 0x7F);      // multiplex ratio = 1/128
    cmd(d, 0xA4);                    // entire display follows RAM
    cmd(d, 0xA6);                    // normal (non-inverted)
    sh1107_clear(d);
    flush_fb(d);
    cmd(d, 0xAF);                    // display on
    bus_unlock(d);
    LOG("[OLED] %s SH1107 ready\n", d->name ? d->name : "?");
}

void sh1107_clear(sh1107_t *d) { memset(d->fb, 0, sizeof(d->fb)); }

// Write a pixel in raw panel coordinates.
static void raw_pixel(sh1107_t *d, int px, int py, bool on) {
    if (px < 0 || py < 0 || px >= SH1107_W || py >= SH1107_H) return;
    int off = (py >> 3) * SH1107_W + px;
    uint8_t bit = (uint8_t)(1u << (py & 7));
    if (on) d->fb[off] |= bit;
    else    d->fb[off] &= (uint8_t)~bit;
}

// Map logical coordinates (what LVGL / the test pattern draw in) to panel
// coordinates: first a cyclic shift (wraps), then the panel rotation. Square
// panel so W==H.
void sh1107_set_pixel(sh1107_t *d, int x, int y, bool on) {
    if (x < 0 || y < 0 || x >= SH1107_W || y >= SH1107_H) return;
    // Cyclic shift in logical space — wraps so nothing is clipped.
    x = ((x + d->x_shift) % SH1107_W + SH1107_W) % SH1107_W;
    y = ((y + d->y_shift) % SH1107_H + SH1107_H) % SH1107_H;
    int px, py;
    switch (d->rotation) {
        case 90:  px = SH1107_H - 1 - y; py = x;              break;
        case 180: px = SH1107_W - 1 - x; py = SH1107_H - 1 - y; break;
        case 270: px = y;                py = SH1107_W - 1 - x; break;
        default:  px = x;                py = y;              break; // 0
    }
    raw_pixel(d, px, py, on);
}

void sh1107_fill_rect(sh1107_t *d, int x, int y, int w, int h, bool on) {
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            sh1107_set_pixel(d, x + i, y + j, on);
}

// Calibration pattern, drawn in logical coordinates so it reflects the final
// (post-rotation) orientation. Corner dot-counts: TL=1, TR=2, BL=3, BR=4.
void sh1107_test_pattern(sh1107_t *d) {
    sh1107_clear(d);
    const int W = SH1107_W, H = SH1107_H;

    // Outer frame (1px) — reveals any shift/clipping at the glass edges.
    sh1107_fill_rect(d, 0, 0, W, 1, true);
    sh1107_fill_rect(d, 0, H - 1, W, 1, true);
    sh1107_fill_rect(d, 0, 0, 1, H, true);
    sh1107_fill_rect(d, W - 1, 0, 1, H, true);
    // Inner frame, inset 6px.
    sh1107_fill_rect(d, 6, 6, W - 12, 1, true);
    sh1107_fill_rect(d, 6, H - 7, W - 12, 1, true);
    sh1107_fill_rect(d, 6, 6, 1, H - 12, true);
    sh1107_fill_rect(d, W - 7, 6, 1, H - 12, true);

    // Diagonal top-left -> bottom-right.
    for (int i = 0; i < (W < H ? W : H); i++) sh1107_set_pixel(d, i, i, true);

    // Centre crosshair.
    sh1107_fill_rect(d, W / 2 - 6, H / 2, 13, 1, true);
    sh1107_fill_rect(d, W / 2, H / 2 - 6, 1, 13, true);

    // Corner dot markers: count = which corner (1/2/3/4).
    const int s = 7, m = 4, g = 4;     // dot size, margin, gap
    // TL: 1
    sh1107_fill_rect(d, m, m, s, s, true);
    // TR: 2
    sh1107_fill_rect(d, W - m - s, m, s, s, true);
    sh1107_fill_rect(d, W - m - 2 * s - g, m, s, s, true);
    // BL: 3
    for (int k = 0; k < 3; k++)
        sh1107_fill_rect(d, m + k * (s + g), H - m - s, s, s, true);
    // BR: 4
    for (int k = 0; k < 4; k++)
        sh1107_fill_rect(d, W - m - s - k * (s + g), H - m - s, s, s, true);

    sh1107_show(d);
}

void sh1107_show(sh1107_t *d) {
    bus_lock(d);
    flush_fb(d);
    bus_unlock(d);
}

// Map a logical point to the panel row (py), matching sh1107_set_pixel's transform.
static int logical_to_py(sh1107_t *d, int x, int y) {
    x = ((x + d->x_shift) % SH1107_W + SH1107_W) % SH1107_W;
    y = ((y + d->y_shift) % SH1107_H + SH1107_H) % SH1107_H;
    switch (d->rotation) {
        case 90:  return x;
        case 180: return SH1107_H - 1 - y;
        case 270: return SH1107_W - 1 - x;
        default:  return y;
    }
}

// Flush only the pages that a logical rectangle touches. NOTE: assumes the area
// does not wrap across the panel edge in the page axis (true for this build's
// rotation/shift) — otherwise it just flushes the spanned page range.
void sh1107_show_area(sh1107_t *d, int lx1, int ly1, int lx2, int ly2) {
    int py[4] = {
        logical_to_py(d, lx1, ly1), logical_to_py(d, lx2, ly1),
        logical_to_py(d, lx1, ly2), logical_to_py(d, lx2, ly2),
    };
    int pmin = py[0], pmax = py[0];
    for (int i = 1; i < 4; i++) { if (py[i] < pmin) pmin = py[i]; if (py[i] > pmax) pmax = py[i]; }
    if (pmin < 0) pmin = 0;
    if (pmax > SH1107_H - 1) pmax = SH1107_H - 1;
    int g0 = pmin / 8, g1 = pmax / 8;

    bus_lock(d);
    uint8_t line[1 + SH1107_W];
    line[0] = 0x40;
    for (int page = g0; page <= g1; page++) {
        cmd(d, 0xB0 | page);
        cmd(d, 0x00);
        cmd(d, 0x10);
        memcpy(&line[1], &d->fb[page * SH1107_W], SH1107_W);
        i2c_write_timeout_us(d->i2c, d->addr, line, sizeof(line), false, 20000);
    }
    bus_unlock(d);
}
