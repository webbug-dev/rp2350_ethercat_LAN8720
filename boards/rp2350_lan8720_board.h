// rp2350_lan8720_board.h — board definition for an RP2354A carrying a LAN8720
// RMII Ethernet module, a 1.5" SH1107 128x128 I2C OLED and two buttons. Included
// by the Pico SDK because PICO_BOARD is set to "rp2350_lan8720_board" and boards/
// is on PICO_BOARD_HEADER_DIRS.
//
// The RP2354A is an RP2350A die with 2 MB of stacked internal QSPI flash, so
// we layer on top of the stock pico2 (RP2350A) board header and only override
// the flash size.  The application pin map lives in src/board.h.
#ifndef _BOARDS_RP2350_LAN8720_BOARD_H
#define _BOARDS_RP2350_LAN8720_BOARD_H

// RP2354A: 2 MB internal stacked flash (vs 4 MB external on a stock Pico 2).
// Define before including pico2.h so its #ifndef guard does not clobber this.
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif

// Mark the RP2354 part (informational; same A2-stepping silicon as RP2350A).
#ifndef PICO_RP2354A
#define PICO_RP2354A 1
#endif

// Reuse everything else from the standard Pico 2 (RP2350A) board header
// (RP2350A, LED on GPIO 25, default UART, etc.).
#include "boards/pico2.h"

#endif
