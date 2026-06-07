// lv_conf.h — LVGL 9.2 configuration for the SH1107 128x128 monochrome OLED.
//
// 8-bpp greyscale internal rendering; the SH1107 flush callback thresholds
// each pixel at the midpoint to 1 bit (see src/display_task.c). DIRECT render
// mode with a single full-frame buffer keeps the flush logic trivial.
#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

#define LV_COLOR_DEPTH                  8
#define LV_COLOR_16_SWAP                0

/* ── Memory ──────────────────────────────────────────────────────────────── */
#define LV_USE_STDLIB_MALLOC            LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_STRING            LV_STDLIB_BUILTIN
#define LV_USE_STDLIB_SPRINTF           LV_STDLIB_BUILTIN
#ifndef LV_MEM_SIZE_KILOBYTES
#  define LV_MEM_SIZE_KILOBYTES         64
#endif
#define LV_MEM_SIZE                     (LV_MEM_SIZE_KILOBYTES * 1024U)

/* ── OS / tick ──────────────────────────────────────────────────────────── */
#define LV_USE_OS                       LV_OS_NONE
#define LV_DEF_REFR_PERIOD              30
#define LV_DPI_DEF                      130

/* ── Rendering ──────────────────────────────────────────────────────────── */
#define LV_USE_DRAW_SW                  1
#define LV_DRAW_SW_COMPLEX              1
// Cortex-M33 on RP2350 has no Helium/MVE — force the plain C blend path so
// LVGL doesn't try to assemble lv_blend_helium.S.
#define LV_DRAW_SW_ASM                  LV_DRAW_SW_ASM_NONE

/* ── Fonts ──────────────────────────────────────────────────────────────── */
#define LV_FONT_MONTSERRAT_10           1
#define LV_FONT_MONTSERRAT_12           1
#define LV_FONT_MONTSERRAT_14           1
#define LV_FONT_MONTSERRAT_16           1
#define LV_FONT_MONTSERRAT_20           1
#define LV_FONT_MONTSERRAT_48           1   // big centred digit on OLED #2
#define LV_FONT_UNSCII_8                1
#define LV_FONT_DEFAULT                 &lv_font_montserrat_14

/* ── Widgets used by display_task ───────────────────────────────────────── */
#define LV_USE_LABEL                    1
#define LV_LABEL_TEXT_SELECTION         0
#define LV_USE_BUTTON                   1
#define LV_USE_BAR                      1
#define LV_USE_LINE                     1
#define LV_USE_LED                      1
#define LV_USE_SPINNER                  1
#define LV_USE_ARC                      1

/* ── Themes ─────────────────────────────────────────────────────────────── */
#define LV_USE_THEME_DEFAULT            1
#define LV_THEME_DEFAULT_DARK           1
#define LV_USE_THEME_SIMPLE             1
#define LV_USE_THEME_MONO               1

/* ── Trim everything we don't need ──────────────────────────────────────── */
#define LV_USE_LOG                      0
#define LV_USE_ASSERT_NULL              0
#define LV_USE_ASSERT_MALLOC            0
#define LV_USE_PERF_MONITOR             0
#define LV_USE_MEM_MONITOR              0
#define LV_USE_FREETYPE                 0
#define LV_USE_FS_STDIO                 0
#define LV_USE_FS_POSIX                 0

#endif /* LV_CONF_H */
