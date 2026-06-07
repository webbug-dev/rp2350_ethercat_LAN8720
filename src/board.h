// board.h — application pin map + EtherCAT config for the
// "RP2354 (2 MB) + LAN8720 (RMII) + 1×SH1107" build.
//
// The LAN8720 is an RMII Ethernet PHY (no MAC). The RP2350 implements the MAC in
// PIO + DMA (src/rmii_mac.c, src/pio/lan8720_*.pio — edge-synced to REF_CLK,
// after messani/pico-lan8720) and runs a small hand-rolled EtherCAT master
// (EtherType 0x88A4) on top. No TCP/IP — just raw Layer-2 EtherCAT frames.
//
//  ┌──────────────────────────────────────────────────────────────────────┐
//  │  WIRING  (RP2354A — GPIO numbers)                                      │
//  ├──────────────────────────────────────────────────────────────────────┤
//  │  LAN8720 module (RMII)   RP2354                                        │
//  │   TXD0          ──────►  GP10   PIO TX                                 │
//  │   TXD1          ──────►  GP11   PIO TX                                 │
//  │   TXEN          ──────►  GP12   PIO TX-enable                          │
//  │   RXD0          ──────►  GP6    PIO RX                                 │
//  │   RXD1          ──────►  GP7    PIO RX                                 │
//  │   CRS_DV        ──────►  GP8    PIO RX data-valid                      │
//  │   MDIO          ──────►  GP14   PHY management data                    │
//  │   MDC           ──────►  GP15   PHY management clock (bit-bang)        │
//  │   nINT/RETCLK   ──────►  GP22   50 MHz REF_CLK OUT from the module     │
//  │                                  (this is the module's clock pin —     │
//  │                                   silk "nINT/RETCLK"/"REFCLK")          │
//  │   nRST (if present) ──►  3V3    tie high; many modules have no RST pin  │
//  │                                                                        │
//  │  SH1107 OLED 128×128     GP4 = I2C0 SDA, GP5 = I2C0 SCL               │
//  │  Buttons (active-low, to GND)  GP2 = SERVO run, GP3 = STEP run        │
//  └──────────────────────────────────────────────────────────────────────┘
//
// Everything runs on 3.3 V — do NOT feed 5 V into the module's 3V3 pin. The RMII
// bus is 3.3 V signalling on this build (no 1.8 V pad threshold).
//
// NOTE: the RMII GPIO numbers above are fixed in src/pio/lan8720_rx.pio (the PIO
// assembler needs them at compile time). The values here are a wiring reference
// and must match that file.
#pragma once

// ── System clock ────────────────────────────────────────────────────────────
// The module supplies REF_CLK; the TX/RX PIO re-sync to it on every di-bit, so
// the system clock just needs to be fast enough to catch the 50 MHz edges. 250
// MHz (clkdiv=1 PIO) matches messani/pico-lan8720. Set in main() before the
// scheduler; configCPU_CLOCK_HZ in FreeRTOSConfig.h must match.
#define SYS_CLOCK_KHZ    250000

// Core-voltage boost for the overclock.
//   CORE_VREG_BOOST   1 = raise the core voltage before overclocking (default).
//                     0 = leave it at the SDK default — try this if you suspect
//                         heat/stability issues and want to test a lower voltage.
//   CORE_VREG_VOLTAGE the VREG_VOLTAGE_* level to apply when the boost is on.
//                     1_15/1_20 are usually plenty for 250 MHz; 1_30 is the SDK
//                     maximum and is what this build ships with.
#define CORE_VREG_BOOST    1
#define CORE_VREG_VOLTAGE  VREG_VOLTAGE_1_30   // SDK max (VREG_VOLTAGE_MAX)

// ── RMII pin reference (authoritative copy is the .pio — keep in sync) ───────
#define RMII_TX0_PIN     10   // TX0, TX1, TXEN are consecutive (10,11,12)
#define RMII_RX0_PIN     6    // RX0, RX1, CRS_DV are consecutive (6,7,8)
#define RMII_MDIO_PIN    14
#define RMII_MDC_PIN     15
#define RMII_REFCLK_PIN  22   // REF_CLK input from the module

// ── SH1107 128x128 OLED (single panel) ──────────────────────────────────────
#define OLED_W           128
#define OLED_H           128
#define OLED_I2C_HZ      (400 * 1000)          // 400 kHz fast-mode I2C
#define OLED_I2C_ADDR    0x3C                  // common SH1107 addr (try 0x3D if blank)
#define OLED_I2C_INST    i2c0
#define OLED_SDA_PIN     4
#define OLED_SCL_PIN     5

// ── Display calibration ──────────────────────────────────────────────────────
#define SH1107_ROTATION   90
#define SH1107_X_SHIFT    0
#define SH1107_Y_SHIFT    32
#define DISPLAY_SELFTEST  0

// ── Buttons (active-low, internal pull-up; press = short to GND) ─────────────
// BTN0 → run the Servo drive, BTN1 → run the Step drive. Hold to run.
#define BTN_COUNT        2
#define BTN0_PIN         2
#define BTN1_PIN         3

// ── Verbose EtherCAT / MAC trace ─────────────────────────────────────────────
// When 0, NET_DBG() compiles to nothing; failures/warnings (LOGE/LOGW) always
// emit. See src/net_debug.h.
#define NET_DEBUG_ENABLE 1

// ── EtherCAT master (hand-rolled, on the LAN8720 raw MAC) ────────────────────
// EtherCAT has no IP layer: destination is broadcast, EtherType 0x88A4.
#define EC_ETHERTYPE       0x88A4

// Two drives are expected on the segment, in auto-increment (wiring) order:
//   position 0 = ServoDrive (Lichuan A5/A8 servo, see doc/servo/)
//   position 1 = StepDrive  (LC10E stepper)
#define EC_DRIVE_COUNT     2
#define EC_DRV_SERVO       0
#define EC_DRV_STEP        1

// Configured station addresses assigned to slaves (position p → BASE+p).
#define EC_STATION_BASE    0x1001

// Mailbox / process-data placement in the slave ESC DPRAM (fallbacks; mailbox
// offsets/sizes are read from each slave's SII at bring-up). SM2/3 start
// addresses come from the ESI <Sm> StartAddress (match the LC10E ESI in doc/).
#define EC_MBX_OUT_ADDR    0x1000   // SM0 (master→slave mailbox)
#define EC_MBX_IN_ADDR     0x1100   // SM1 (slave→master mailbox)
#define EC_MBX_SIZE        0x0080
#define EC_SM2_ADDR        0x1200   // outputs (RxPDO)
#define EC_SM3_ADDR        0x1300   // inputs  (TxPDO)

// Optional product-code match (CoE 0x1018:02) to decide which slave is the
// Servo / Step. Leave 0 to fall back to wiring order (position 0 = Servo).
// From the ESI files in doc/: vendor 0x0A79; CL3-E57H (LC-EH family) 0x1000.
#define EC_SERVO_PRODUCT   0x00000000u
#define EC_STEP_PRODUCT    0x00001000u   // CL3-E57H → StepDrive / GP3

// Cyclic process-data period (ms). The PIO RMII MAC has no per-frame SPI
// round-trip, so 1 ms (~1 kHz) is comfortable for this CSP velocity demo.
#define EC_CYCLE_MS        1

// Velocity profile (CiA-402 Cyclic-Sync-Position). Hold a button to ramp up;
// the ramp eases in and reaches EC_TARGET_RPM at EC_RAMP_UP_MS. Releasing ramps
// smoothly back down to a stop.
#define EC_TARGET_RPM      1500
#define EC_RAMP_UP_MS      5000
#define EC_RAMP_DOWN_MS    670

// Autonomous bench test: ignore the buttons and spin every drive that reaches
// OP at EC_AUTORUN_RPM. Set 0 for normal button-driven operation.
#define EC_AUTORUN         0
#define EC_AUTORUN_RPM     50

// Drive feedback resolution — counts per motor revolution (unit of CiA-402
// position 0x6064 / target position 0x607A). CL3-E57H defaults to 10000.
#define EC_COUNTS_PER_REV  10000L

// Locally-administered MAC (EtherCAT source address; a raw MAC has no address
// filter).
#define ETH_MAC0 0x02
#define ETH_MAC1 0x08
#define ETH_MAC2 0xDC
#define ETH_MAC3 0x53
#define ETH_MAC4 0x35
#define ETH_MAC5 0x00
