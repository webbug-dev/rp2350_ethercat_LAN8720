#include "display_task.h"
#include "shared_state.h"
#include "sh1107.h"
#include "periph.h"
#include "board.h"
#include "log.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "FreeRTOS.h"
#include "task.h"

#include "lvgl.h"

// I2C presence probe: write a 0x00 (NOP command prefix) and see if the device
// ACKs. Used to diagnose a blank panel (wrong addr / wiring / pull-ups).
static bool i2c_acks(uint8_t addr) {
    uint8_t z = 0;
    xSemaphoreTake(g_oled_mutex, portMAX_DELAY);
    int r = i2c_write_timeout_us(OLED_I2C_INST, addr, &z, 1, false, 2000);
    xSemaphoreGive(g_oled_mutex);
    return r >= 0;
}

// Probe one (instance, SDA, SCL) candidate for an SH1107 (0x3C/0x3D), then
// release the pins. Used by the locator to find a re-wired panel.
static bool probe_pair(i2c_inst_t *inst, uint sda, uint scl) {
    i2c_init(inst, 100 * 1000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda); gpio_pull_up(scl);
    sleep_ms(3);
    uint8_t z = 0;
    bool ok = i2c_write_timeout_us(inst, 0x3C, &z, 1, false, 2000) >= 0 ||
              i2c_write_timeout_us(inst, 0x3D, &z, 1, false, 2000) >= 0;
    gpio_disable_pulls(sda); gpio_disable_pulls(scl);
    gpio_set_function(sda, GPIO_FUNC_SIO);
    gpio_set_function(scl, GPIO_FUNC_SIO);
    return ok;
}

// Sweep every free I2C-capable pin pair to locate a re-wired OLED, then restore
// the configured bus (i2c0 @ GP4/5). Avoids pins owned by the RMII MAC (GP6-15, 22),
// the buttons (GP2/3) and the USB. Runs once.
static void oled_locate(void) {
    static const struct { i2c_inst_t *i; uint sda, scl; const char *l; } c[] = {
        { i2c0, 4,  5,  "i2c0 GP4/GP5"   }, { i2c0, 0,  1,  "i2c0 GP0/GP1"   },
        { i2c0, 8,  9,  "i2c0 GP8/GP9"   }, { i2c0, 12, 13, "i2c0 GP12/GP13" },
        { i2c1, 6,  7,  "i2c1 GP6/GP7"   }, { i2c1, 10, 11, "i2c1 GP10/GP11" },
        { i2c1, 14, 15, "i2c1 GP14/GP15" }, { i2c1, 22, 23, "i2c1 GP22/GP23" },
        { i2c1, 26, 27, "i2c1 GP26/GP27" },
    };
    LOGW("[OLED] locating panel across free I2C pins...\n");
    int found = 0;
    for (unsigned k = 0; k < sizeof(c) / sizeof(c[0]); k++)
        if (probe_pair(c[k].i, c[k].sda, c[k].scl)) { LOGW("[OLED]   >>> FOUND on %s <<<\n", c[k].l); found++; }
    if (!found) LOGW("[OLED]   nothing anywhere — panel unpowered / not wired / no pull-ups\n");
    // Restore the configured OLED bus.
    i2c_init(OLED_I2C_INST, OLED_I2C_HZ);
    gpio_set_function(OLED_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(OLED_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(OLED_SDA_PIN); gpio_pull_up(OLED_SCL_PIN);
}

// ── One SH1107 128×128 panel, split into two stacked windows ─────────────────
static sh1107_t s_oled = {
    .i2c = OLED_I2C_INST, .sda_pin = OLED_SDA_PIN, .scl_pin = OLED_SCL_PIN,
    .i2c_hz = OLED_I2C_HZ, .addr = OLED_I2C_ADDR, .rotation = SH1107_ROTATION,
    .x_shift = SH1107_X_SHIFT, .y_shift = SH1107_Y_SHIFT, .name = "OLED",
};

// ── LVGL → SH1107 plumbing ──────────────────────────────────────────────────
#define FB_W OLED_W
#define FB_H OLED_H
static uint8_t s_lv_buf[FB_W * FB_H];      // 8-bpp full frame (DIRECT render)

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    sh1107_t *d = (sh1107_t *)lv_display_get_user_data(disp);
    int x1 = area->x1 < 0 ? 0 : area->x1;
    int y1 = area->y1 < 0 ? 0 : area->y1;
    int x2 = area->x2 >= FB_W ? FB_W - 1 : area->x2;
    int y2 = area->y2 >= FB_H ? FB_H - 1 : area->y2;
    for (int y = y1; y <= y2; y++)
        for (int x = x1; x <= x2; x++)
            sh1107_set_pixel(d, x, y, px_map[y * FB_W + x] >= 0x80);
    sh1107_show_area(d, x1, y1, x2, y2);   // flush only the changed pages (fast)
    lv_display_flush_ready(disp);
}

static uint32_t hw_tick_ms(void) { return to_ms_since_boot(get_absolute_time()); }

// Classic Mac is black-on-white: lit pixels form the window/frames, dark the text.
#define COL_W lv_color_white()     // lit
#define COL_B lv_color_black()     // dark

// ── Desktop 50% dither (2×2 checkerboard, tiled over the screen) ────────────
static const uint8_t s_dither_map[4] = { 0xFF, 0x00, 0x00, 0xFF };
static const lv_image_dsc_t s_dither = {
    .header = { .magic = LV_IMAGE_HEADER_MAGIC, .cf = LV_COLOR_FORMAT_L8,
                .w = 2, .h = 2, .stride = 2 },
    .data_size = 4, .data = s_dither_map,
};

// ── Small builders ───────────────────────────────────────────────────────────
static lv_obj_t *box(lv_obj_t *p, int x, int y, int w, int h, int r,
                     lv_color_t bg, bool border) {
    lv_obj_t *o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, h);
    lv_obj_set_style_radius(o, r, 0);
    lv_obj_set_style_bg_color(o, bg, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    if (border) {
        lv_obj_set_style_border_color(o, COL_B, 0);
        lv_obj_set_style_border_width(o, 1, 0);
        lv_obj_set_style_border_opa(o, LV_OPA_COVER, 0);
    }
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    return o;
}
static void hline(lv_obj_t *p, int x, int y, int w, lv_color_t c) {
    lv_obj_t *o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_pos(o, x, y);
    lv_obj_set_size(o, w, 1);
    lv_obj_set_style_bg_color(o, c, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_COVER, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
}
static lv_obj_t *text(lv_obj_t *p, const lv_font_t *f, lv_color_t c) {
    lv_obj_t *l = lv_label_create(p);
    lv_obj_set_style_text_color(l, c, 0);
    lv_obj_set_style_text_font(l, f, 0);
    return l;
}

// Compact pinstriped title bar with a close box (tb_h px tall).
static void title_bar(lv_obj_t *win, int winw, int tb_h, const char *name) {
    box(win, 1, 1, winw - 3, tb_h, 6, COL_W, false);      // clears separator
    for (int y = 4; y <= tb_h - 3; y += 2)                 // pinstripes
        hline(win, 8, y, winw - 16, COL_B);
    hline(win, 1, tb_h + 1, winw - 2, COL_B);              // bar separator
    box(win, 3, 2, tb_h - 2, tb_h - 2, 0, COL_W, false);   // clear around close box
    box(win, 5, 4, tb_h - 6, tb_h - 6, 2, COL_W, true);    // close box (hollow)
    lv_obj_t *t = text(win, &lv_font_montserrat_12, COL_B);
    lv_obj_set_style_bg_color(t, COL_W, 0);                // white gap in stripes
    lv_obj_set_style_bg_opa(t, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(t, 4, 0);
    lv_label_set_text(t, name);
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 0);
}

// ── Per-drive window (one of two stacked on the single panel) ────────────────
typedef struct {
    lv_obj_t *status;   // "searching" / "online"
    lv_obj_t *track;    // indeterminate-sweep track (searching only)
    lv_obj_t *pill;     // indeterminate-sweep moving segment
    lv_obj_t *bar;      // speed gauge (running) — kept but unused
    lv_obj_t *speed;    // rpm readout (running)
    lv_obj_t *info;     // position line
    lv_obj_t *sq[8];    // online: two 2x2 chasers (0..3 left, 4..7 right)
} win_t;
static win_t s_win[EC_DRIVE_COUNT];

// Two windows with a 5px margin on every screen edge and a 5px gap between them.
#define MARGIN   5
#define WIN_GAP  5
#define TB_H     13                          // title-bar height
#define WIN_W    (FB_W - 2 * MARGIN)         // window width  (118)
#define WIN_H    ((FB_H - 2 * MARGIN - WIN_GAP) / 2)  // window height (56)
#define WIN0_Y   MARGIN                       // top window y   (5)
#define WIN1_Y   (MARGIN + WIN_H + WIN_GAP)   // bottom window y (66)
#define C1_Y     (TB_H + 4)                   // content line 1
#define C2_Y     (TB_H + 19)                  // content line 2
#define ANIM_Y   (TB_H + 33)                  // bottom sweep row
#define TRACK_X  6
#define TRACK_W  (WIN_W - 12)                 // 106

static uint32_t s_anim;            // animation phase, advanced each refresh

// Windows-style indeterminate sweep: a segment glides left→right with ease-in /
// ease-out, then restarts from the left. Returns its x offset within [0,travel].
static int sweep(int travel) {
    if (travel < 1) travel = 1;
    const uint32_t period = 60;                          // ~1.2 s per sweep
    float t = (float)(s_anim % period) / (float)period;  // 0..1
    float e = t * t * (3.0f - 2.0f * t);                 // smoothstep ease
    return (int)(e * (float)travel);
}
static void show(lv_obj_t *o, bool v) {
    if (!o) return;                      // tolerate a NULL object (e.g. LVGL OOM)
    if (v) lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
    else   lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}

static void build_window(lv_obj_t *scr, int d, const char *name, int wy) {
    box(scr, MARGIN + 1, wy + 1, WIN_W, WIN_H, 7, COL_B, false);  // drop shadow
    lv_obj_t *win = box(scr, MARGIN, wy, WIN_W, WIN_H, 7, COL_W, true);
    title_bar(win, WIN_W, TB_H, name);

    win_t *w = &s_win[d];

    w->status = text(win, &lv_font_montserrat_14, COL_B);
    lv_label_set_text(w->status, "Searching");

    // Indeterminate-sweep track + moving pill (Windows-style).
    w->track = box(win, TRACK_X, C2_Y, TRACK_W, 7, 3, COL_W, true);
    w->pill  = box(win, TRACK_X + 1, C2_Y + 1, 34, 5, 2, COL_B, false);

    // Speed gauge — wide and tall, with the rpm drawn inside it.
    w->bar = lv_bar_create(win);
    lv_obj_remove_style_all(w->bar);
    lv_obj_set_size(w->bar, WIN_W - 8, 18);
    lv_obj_align(w->bar, LV_ALIGN_TOP_MID, 0, C1_Y);
    lv_obj_set_style_radius(w->bar, 3, LV_PART_MAIN);
    lv_obj_set_style_bg_color(w->bar, COL_W, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(w->bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(w->bar, COL_B, LV_PART_MAIN);
    lv_obj_set_style_border_width(w->bar, 1, LV_PART_MAIN);
    lv_obj_set_style_border_opa(w->bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(w->bar, 2, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(w->bar, COL_B, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(w->bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_bar_set_range(w->bar, 0, EC_TARGET_RPM);
    lv_bar_set_value(w->bar, 0, LV_ANIM_OFF);
    lv_obj_add_flag(w->bar, LV_OBJ_FLAG_HIDDEN);

    // rpm readout while running — plain bold text, no bar behind it.
    w->speed = text(win, &lv_font_montserrat_16, COL_B);
    lv_label_set_text(w->speed, "0 rpm");
    lv_obj_add_flag(w->speed, LV_OBJ_FLAG_HIDDEN);

    w->info = text(win, &lv_font_montserrat_12, COL_B);
    lv_label_set_text(w->info, "0.00 r");
    lv_obj_add_flag(w->info, LV_OBJ_FLAG_HIDDEN);

    // online animation: two 2x2 chasers (a "block of 4 squares" each), flanking
    // the text. Fixed positions; only their fill toggles while running.
    const int SQ = 5, GAP = 1, BLK = SQ + GAP;     // 2x2 block = 11 px
    const int LY = C1_Y, LX = 6, RX = WIN_W - 6 - (SQ + BLK);
    const int ox[4] = { 0, BLK, BLK, 0 };          // TL, TR, BR, BL (clockwise)
    const int oy[4] = { 0, 0, BLK, BLK };
    for (int i = 0; i < 4; i++) {                   // left block
        w->sq[i]     = box(win, LX + ox[i], LY + oy[i], SQ, SQ, 1, COL_W, true);
        lv_obj_add_flag(w->sq[i], LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < 4; i++) {                   // right block
        w->sq[4 + i] = box(win, RX + ox[i], LY + oy[i], SQ, SQ, 1, COL_W, true);
        lv_obj_add_flag(w->sq[4 + i], LV_OBJ_FLAG_HIDDEN);
    }
}

// Encoder counts → "s.ss r" (revolutions, signed, 2 decimals).
static void fmt_rev(char *buf, size_t cap, int32_t counts) {
    long rev100 = (long)((int64_t)counts * 100 / EC_COUNTS_PER_REV);
    const char *sgn = rev100 < 0 ? "-" : "";
    long a = rev100 < 0 ? -rev100 : rev100;
    snprintf(buf, cap, "%s%ld.%02ld", sgn, a / 100, a % 100);
}

// Position + size the indeterminate sweep for this state, and run it.
static void sweep_anim(win_t *w, int y, int h, int pill_w, int pill_h) {
    lv_obj_set_pos(w->track, TRACK_X, y);
    lv_obj_set_size(w->track, TRACK_W, h);
    lv_obj_set_size(w->pill, pill_w, pill_h);
    lv_obj_set_pos(w->pill, TRACK_X + 1 + sweep(TRACK_W - 2 - pill_w), y + (h - pill_h) / 2);
}

static void refresh_window(int d) {
    SHARED_LOCK();
    drive_state_t s = g_shared.drive[d];
    SHARED_UNLOCK();
    win_t *w = &s_win[d];

    if (s.phase == DS_RUNNING) {
        // Just the (measured) speed as text — no progress bar.
        show(w->bar, false); show(w->status, false); show(w->track, false); show(w->pill, false);
        for (int i = 0; i < 8; i++) show(w->sq[i], false);
        show(w->speed, true); show(w->info, true);
        char sb[16]; snprintf(sb, sizeof(sb), "%d rpm", s.velocity_rpm);
        lv_label_set_text(w->speed, sb);
        lv_obj_align(w->speed, LV_ALIGN_TOP_MID, 0, C1_Y);
        char rev[16]; fmt_rev(rev, sizeof(rev), s.position);
        lv_label_set_text(w->info, rev);
        lv_obj_align(w->info, LV_ALIGN_TOP_MID, 0, C2_Y + 4);
    } else if (s.phase == DS_CONNECTED) {
        // "online" + position, flanked by two 2x2 chasers; right runs anti-phase.
        show(w->bar, false); show(w->speed, false); show(w->track, false); show(w->pill, false);
        show(w->status, true); show(w->info, true);
        for (int i = 0; i < 8; i++) show(w->sq[i], true);
        lv_obj_set_style_text_font(w->status, &lv_font_montserrat_12, 0);
        lv_label_set_text(w->status, s.model[0] ? s.model : "online");
        lv_obj_align(w->status, LV_ALIGN_TOP_MID, 0, C1_Y);
        char rev[16]; fmt_rev(rev, sizeof(rev), s.position);
        lv_label_set_text(w->info, rev);
        lv_obj_align(w->info, LV_ALIGN_TOP_MID, 0, C2_Y);
        int step  = (int)((s_anim / 3) % 4);           // active square ~40 ms each
        int stepR = (step + 2) % 4;                    // right in anti-phase
        for (int i = 0; i < 4; i++) {
            lv_obj_set_style_bg_color(w->sq[i],     i == step  ? COL_B : COL_W, 0);
            lv_obj_set_style_bg_color(w->sq[4 + i], i == stepR ? COL_B : COL_W, 0);
        }
    } else {
        // searching: small label + Windows-style sweep.
        show(w->bar, false); show(w->speed, false); show(w->info, false);
        for (int i = 0; i < 8; i++) show(w->sq[i], false);
        show(w->status, true); show(w->track, true); show(w->pill, true);
        lv_obj_set_style_text_font(w->status, &lv_font_montserrat_12, 0);
        lv_label_set_text(w->status, "searching");
        lv_obj_align(w->status, LV_ALIGN_TOP_MID, 0, C1_Y);
        sweep_anim(w, C2_Y + 2, 7, 34, 5);                         // prominent
    }
}

static void display_task(void *arg) {
    (void)arg;
    LOG("[DISP] display task on core %u\n", (unsigned)get_core_num());

    s_oled.bus_mutex = g_oled_mutex;
    sh1107_init(&s_oled);

#if DISPLAY_SELFTEST
    LOG("[DISP] SELF-TEST pattern\n");
    for (;;) { sh1107_test_pattern(&s_oled); vTaskDelay(pdMS_TO_TICKS(500)); }
#endif

    lv_init();
    lv_tick_set_cb(hw_tick_ms);

    lv_display_t *disp = lv_display_create(FB_W, FB_H);
    lv_display_set_user_data(disp, &s_oled);
    lv_display_set_buffers(disp, s_lv_buf, NULL, FB_W * FB_H,
                           LV_DISPLAY_RENDER_MODE_DIRECT);
    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_set_theme(disp, lv_theme_mono_init(disp, false, &lv_font_montserrat_14));

    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_set_style_bg_color(scr, COL_B, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_image_src(scr, &s_dither, 0);
    lv_obj_set_style_bg_image_tiled(scr, true, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);

    build_window(scr, EC_DRV_SERVO, "ServoDrive", WIN0_Y);
    build_window(scr, EC_DRV_STEP,  "StepDrive",  WIN1_Y);
    LOG("[DISP] LVGL UI built (ServoDrive + StepDrive)\n");

    uint32_t last_refresh = 0, last_probe = 0;
    bool panel_ok = false;
    for (;;) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // The panel may answer only after the bus settles (its boot-time init can
        // NAK and leave it dark). Poll for it ONLY until it first appears, then
        // (re)initialize it and stop probing — no steady-state bus noise. The
        // first pass sweeps every free pin pair to locate a re-wired panel.
        if (!panel_ok && now - last_probe >= 500) {
            last_probe = now;
            static bool located;
            if (!located) { located = true; oled_locate(); }
            if (i2c_acks(0x3C) || i2c_acks(0x3D)) {
                sh1107_init(&s_oled);                        // (re)init now that it answers
                lv_obj_invalidate(lv_display_get_screen_active(disp));  // full redraw
                panel_ok = true;
                LOGW("[OLED] panel detected — initialized at max brightness\n");
            }
        }

        // Buttons are serviced on this core (core 0): publish each drive's run
        // request (active-low button held = run) for the EtherCAT task on core 1.
        // Log the press/release edges so the buttons can be verified from the log.
        static bool prev[EC_DRIVE_COUNT];
        for (int d = 0; d < EC_DRIVE_COUNT; d++) {
            bool down = button_down(d);
            if (down != prev[d]) {
                LOGW("[BTN] %s %s\n", d == EC_DRV_SERVO ? "Servo" : "Step",
                     down ? "pressed (hold to run)" : "released");
                prev[d] = down;
            }
            SHARED_LOCK(); g_shared.drive[d].run_req = down; SHARED_UNLOCK();
        }

        if (now - last_refresh >= 40) {
            s_anim += 3;                       // advance the slide animation
            for (int d = 0; d < EC_DRIVE_COUNT; d++) refresh_window(d);
            last_refresh = now;
        }
        lv_timer_handler();
        watchdog_update();                 // feed the hardware watchdog (core 0)
        vTaskDelay(pdMS_TO_TICKS(10));     // snappier button polling
    }
}

void display_task_start(UBaseType_t priority) {
    TaskHandle_t h = NULL;
    BaseType_t r = xTaskCreate(display_task, "disp", 4096, NULL, priority, &h);
    configASSERT(r == pdPASS);
    vTaskCoreAffinitySet(h, (UBaseType_t)(1u << 0));   // core 0 — UI + buttons
}
