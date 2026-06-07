// rmii_mac.c — raw L2 Ethernet MAC for a LAN8720 RMII module on RP2350 PIO+DMA.
//
// TX/RX PIO are edge-synchronised to the module's REF_CLK (GP22) — derived from
// messani/pico-lan8720, which got 100 Mbit working on an unmodified module by
// re-syncing every di-bit instead of free-running (the free-running NCE external
// TX produced frames the PHY rejected). Runs at clkdiv=1, sys=250 MHz.
//
// Frames are full Ethernet frames (dst+src+ethertype+payload). On TX we prepend
// the preamble+SFD and append the FCS; on RX the PIO strips preamble/SFD and we
// verify+strip the FCS. We send/receive whole Ethernet frames (no FCS), so the
// EtherCAT master in ethercat.c is unchanged apart from the send/recv calls.

#include <string.h>

#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/resets.h"
#include "hardware/sync.h"

#include "pico/stdlib.h"
#include "pico/time.h"

#include "lan8720a.h"
#include "rmii_mac.h"
#include "log.h"
#include "net_debug.h"

#include "lan8720_rx.pio.h"
#include "lan8720_tx.pio.h"

#define PIO_DEV   pio0
#define SM_RX     PICO_RMII_ETHERNET_SM_RX
#define SM_TX     PICO_RMII_ETHERNET_SM_TX

// ── Ethernet CRC32 (FCS) ─────────────────────────────────────────────────────
#define CRC_POLY 0xedb88320u
static uint32_t eth_crc(const uint8_t *data, int len) {
    uint32_t crc = 0xffffffffu;
    while (--len >= 0) {
        uint8_t b = *data++;
        for (int i = 8; --i >= 0; b >>= 1)
            crc = (crc ^ b) & 1 ? (crc >> 1) ^ CRC_POLY : crc >> 1;
    }
    return ~crc;
}

// ── TX ───────────────────────────────────────────────────────────────────────
#define PREAMBLE_SFD 8
static uint8_t  s_txbuf[PREAMBLE_SFD + 1518 + 4] __attribute__((aligned(4)));
static int      s_tx_dma;
static volatile uint32_t s_tx_frames;

int rmii_mac_send(const uint8_t *frame, size_t len) {
    if (len < 14 || len > 1514) return 0;
    dma_channel_wait_for_finish_blocking(s_tx_dma);   // let the previous frame go out

    memcpy(s_txbuf + PREAMBLE_SFD, frame, len);
    size_t flen = len;
    if (flen < 60) { memset(s_txbuf + PREAMBLE_SFD + flen, 0, 60 - flen); flen = 60; }  // pad
    uint32_t crc = eth_crc(s_txbuf + PREAMBLE_SFD, (int)flen);
    memcpy(s_txbuf + PREAMBLE_SFD + flen, &crc, 4);
    size_t total = PREAMBLE_SFD + flen + 4;

    dma_channel_set_read_addr(s_tx_dma, s_txbuf, false);
    dma_channel_set_trans_count(s_tx_dma, total, true);
    s_tx_frames++;
    return (int)len;
}

// ── RX ───────────────────────────────────────────────────────────────────────
#define RX_NBUF   6
#define RX_BUFSZ  1600
static uint8_t  s_rxbuf[RX_NBUF][RX_BUFSZ] __attribute__((aligned(4)));
static volatile uint16_t s_rxlen[RX_NBUF];     // 0 = empty/in-progress, >0 = ready (incl FCS)
static volatile int s_rx_w = 0;                // buffer the DMA is filling (ISR)
static int      s_rx_r = 0;                    // next buffer recv() reads
static int      s_rx_dma;
static volatile uint32_t s_rx_frames, s_rx_isr;
static uint     s_rx_off, s_tx_off;     // PIO program offsets (for restart)

static void __not_in_flash_func(rmii_rx_isr)(void) {
    if (!(PIO_DEV->ints0 & (PIO_IRQ0_INTS_SM0_BITS << SM_RX))) return;
    s_rx_isr++;
    dma_channel_abort(s_rx_dma);
    uint8_t *wr = (uint8_t *)dma_hw->ch[s_rx_dma].write_addr;
    uint16_t n = (uint16_t)(wr - s_rxbuf[s_rx_w]);
    if (n > 16 && n <= RX_BUFSZ) { s_rxlen[s_rx_w] = n; s_rx_frames++; }
    s_rx_w = (s_rx_w + 1) % RX_NBUF;
    s_rxlen[s_rx_w] = 0;
    dma_channel_set_write_addr(s_rx_dma, s_rxbuf[s_rx_w], false);
    dma_channel_set_trans_count(s_rx_dma, RX_BUFSZ, true);
    pio_interrupt_clear(PIO_DEV, SM_RX);          // resume the RX state machine
}

int rmii_mac_recv(uint8_t *buf, size_t cap) {
    uint16_t n = s_rxlen[s_rx_r];
    if (n == 0) return 0;
    uint8_t *src = s_rxbuf[s_rx_r];
    int ret = 0;
    if (n >= 18) {                                // 14 hdr + 4 FCS minimum
        uint32_t crc = eth_crc(src, n - 4);
        if (memcmp(&crc, src + n - 4, 4) == 0) {  // FCS ok
            int payload = n - 4;
            if (payload > (int)cap) payload = (int)cap;
            memcpy(buf, src, payload);
            ret = payload;
        }
    }
    s_rxlen[s_rx_r] = 0;
    s_rx_r = (s_rx_r + 1) % RX_NBUF;
    return ret;
}

void rmii_mac_stats(uint32_t *tx, uint32_t *rx) {
    if (tx) *tx = s_tx_frames;
    if (rx) *rx = s_rx_frames;
}

// Re-arm the RX path from a clean slate. After a link glitch the RX SM can be
// left waiting mid-frame and the DMA mid-write, so subsequent frames are mangled
// (replies come back with WKC=0). Call this before re-scanning the segment.
void rmii_mac_rx_restart(void) {
    uint32_t save = save_and_disable_interrupts();   // the eof ISR touches these
    pio_sm_set_enabled(PIO_DEV, SM_RX, false);
    dma_channel_abort(s_rx_dma);
    pio_sm_clear_fifos(PIO_DEV, SM_RX);
    pio_sm_restart(PIO_DEV, SM_RX);
    pio_interrupt_clear(PIO_DEV, SM_RX);
    for (int i = 0; i < RX_NBUF; i++) s_rxlen[i] = 0;
    s_rx_w = 0; s_rx_r = 0;
    dma_channel_set_write_addr(s_rx_dma, s_rxbuf[0], false);
    dma_channel_set_trans_count(s_rx_dma, RX_BUFSZ, true);
    pio_sm_exec(PIO_DEV, SM_RX, pio_encode_jmp(s_rx_off));
    pio_sm_set_enabled(PIO_DEV, SM_RX, true);
    restore_interrupts(save);
}

// ────────────────────────────────────────────────────────────────────────────
//                    MDIO (Clause-22, synchronous bit-bang)
// ────────────────────────────────────────────────────────────────────────────
#define MDIO_HALF_US 4

static void mdio_idle(void) {
    gpio_set_dir(PICO_RMII_ETHERNET_MDIO_PIN, GPIO_OUT);
    gpio_put(PICO_RMII_ETHERNET_MDIO_PIN, 1);
}
static void mdio_out_bit(int b) {
    gpio_put(PICO_RMII_ETHERNET_MDIO_PIN, b & 1);
    busy_wait_us(MDIO_HALF_US);
    gpio_put(PICO_RMII_ETHERNET_MDC_PIN, 1);
    busy_wait_us(MDIO_HALF_US);
    gpio_put(PICO_RMII_ETHERNET_MDC_PIN, 0);
}
static int mdio_in_bit(void) {
    busy_wait_us(MDIO_HALF_US);
    gpio_put(PICO_RMII_ETHERNET_MDC_PIN, 1);
    busy_wait_us(MDIO_HALF_US);
    int b = gpio_get(PICO_RMII_ETHERNET_MDIO_PIN);
    gpio_put(PICO_RMII_ETHERNET_MDC_PIN, 0);
    return b;
}
static void mdio_out_n(uint32_t val, int n) {
    for (int i = n - 1; i >= 0; i--) mdio_out_bit((val >> i) & 1);
}

static uint16_t mdio_read_once(unsigned phy, unsigned reg) {
    gpio_set_dir(PICO_RMII_ETHERNET_MDIO_PIN, GPIO_OUT);
    mdio_out_n(0xffffffff, 32);
    mdio_out_n(0b01, 2);                   // ST
    mdio_out_n(0b10, 2);                   // OP=read
    mdio_out_n(phy & 0x1f, 5);
    mdio_out_n(reg & 0x1f, 5);
    gpio_set_dir(PICO_RMII_ETHERNET_MDIO_PIN, GPIO_IN);
    mdio_in_bit();                         // turnaround
    uint16_t v = 0;
    for (int i = 0; i < 16; i++) v = (uint16_t)((v << 1) | mdio_in_bit());
    mdio_idle();
    return v;
}
uint16_t rmii_mac_mdio_read(unsigned phy, unsigned reg) {
    for (int t = 0; t < 4; t++) {
        uint16_t v = mdio_read_once(phy, reg);
        if (v != 0xffff) return v;
        for (int k = 0; k < 4; k++) mdio_out_bit(1);
    }
    return 0xffff;
}
static void mdio_write_once(unsigned phy, unsigned reg, unsigned val) {
    gpio_set_dir(PICO_RMII_ETHERNET_MDIO_PIN, GPIO_OUT);
    mdio_out_n(0xffffffff, 32);
    mdio_out_n(0b01, 2);                   // ST
    mdio_out_n(0b01, 2);                   // OP=write
    mdio_out_n(phy & 0x1f, 5);
    mdio_out_n(reg & 0x1f, 5);
    mdio_out_n(0b10, 2);                   // TA
    mdio_out_n(val & 0xffff, 16);
    mdio_idle();
}
void rmii_mac_mdio_write(unsigned phy, unsigned reg, unsigned val) {
    mdio_write_once(phy, reg, val);
    for (int k = 0; k < 4; k++) mdio_out_bit(1);
    mdio_write_once(phy, reg, val);
}

// ── PHY link state (cached) ──────────────────────────────────────────────────
static int s_phy = 0xffff;
static bool s_link = false;
static int  s_speed = 0;
static absolute_time_t s_next_link;

static void link_poll(void) {
    if (s_phy == 0xffff) return;
    uint16_t bs = rmii_mac_mdio_read(s_phy, LAN8720A_BASIC_STATUS_REG);
    if (bs == 0xffff) return;
    bool was = s_link;
    s_link = (bs & LAN8720A_BASIC_STATUS_REG_LINK_STATUS) != 0;
    s_speed = s_link ? 100 : 0;
    // Only log on a change — a per-poll line every 200 ms floods USB-CDC.
    if (s_link != was) LOGW("[MAC] *** LINK %s *** (BMSR=0x%04X)\n", s_link ? "UP" : "DOWN", bs);
}
bool rmii_mac_link_up(void) {
    if (absolute_time_diff_us(get_absolute_time(), s_next_link) < 0) {
        s_next_link = make_timeout_time_ms(200);
        link_poll();
    }
    return s_link;
}
int rmii_mac_link_speed(void) { rmii_mac_link_up(); return s_speed; }
uint16_t rmii_mac_phy_bmsr(void) { return s_phy == 0xffff ? 0xffff : rmii_mac_mdio_read(s_phy, 1); }
int rmii_mac_phy_addr(void) { return s_phy; }

// ── Init ────────────────────────────────────────────────────────────────────
bool rmii_mac_init(const uint8_t mac[6]) {
    (void)mac;
    LOGW("[MAC] LAN8720 init: TX=GP%d/%d/%d RX=GP%d/%d/%d MDIO=GP%d MDC=GP%d REFCLK=GP%d clk_sys=%lu\n",
         PICO_RMII_ETHERNET_TX_PIN, PICO_RMII_ETHERNET_TX_PIN+1, PICO_RMII_ETHERNET_TX_PIN+2,
         PICO_RMII_ETHERNET_RX_PIN, PICO_RMII_ETHERNET_RX_PIN+1, PICO_RMII_ETHERNET_RX_PIN+2,
         PICO_RMII_ETHERNET_MDIO_PIN, PICO_RMII_ETHERNET_MDC_PIN, PICO_RMII_ETHERNET_RETCLK_PIN,
         (unsigned long)clock_get_hz(clk_sys));

    // Soft-restart safety: fully reset the PIO0 and DMA hardware blocks before we
    // touch them, so any state machine / DMA still running from a previous boot
    // (e.g. after a watchdog reset, when peripherals aren't power-cycled) is
    // killed — otherwise leftover transfers corrupt the fresh setup.
    reset_block(RESETS_RESET_PIO0_BITS | RESETS_RESET_DMA_BITS);
    unreset_block_wait(RESETS_RESET_PIO0_BITS | RESETS_RESET_DMA_BITS);

    // REF_CLK input from the module.
    gpio_init(PICO_RMII_ETHERNET_RETCLK_PIN);
    gpio_set_dir(PICO_RMII_ETHERNET_RETCLK_PIN, GPIO_IN);

    // ── MDIO bit-bang pins ──
    gpio_init(PICO_RMII_ETHERNET_MDC_PIN);
    gpio_set_dir(PICO_RMII_ETHERNET_MDC_PIN, GPIO_OUT);
    gpio_put(PICO_RMII_ETHERNET_MDC_PIN, 0);
    gpio_init(PICO_RMII_ETHERNET_MDIO_PIN);
    gpio_set_dir(PICO_RMII_ETHERNET_MDIO_PIN, GPIO_OUT);
    gpio_pull_up(PICO_RMII_ETHERNET_MDIO_PIN);
    gpio_put(PICO_RMII_ETHERNET_MDIO_PIN, 1);
    sleep_ms(50);

    // ── Find the PHY (retry: slow cold-start) ──
    for (int attempt = 0; attempt < 15 && s_phy == 0xffff; attempt++) {
        for (int i = 0; i < 32; i++) {
            if (rmii_mac_mdio_read(i, 2) != 0xffff && rmii_mac_mdio_read(i, 2) != 0) { s_phy = i; break; }
        }
        if (s_phy == 0xffff) { LOGW("[MAC] no PHY yet (%d/15)...\n", attempt+1); sleep_ms(200); }
    }
    if (s_phy == 0xffff) { LOGE("[MAC] NO PHY on MDIO — check wiring/power/REF_CLK\n"); return false; }
    LOGW("[MAC] PHY @addr %d ID=0x%04X%04X\n", s_phy, rmii_mac_mdio_read(s_phy, 2), rmii_mac_mdio_read(s_phy, 3));

    // Soft-reset (clears any stuck state), then advertise 100BASE-TX + restart
    // auto-negotiation (an EtherCAT slave is 100/full).
    rmii_mac_mdio_write(s_phy, 0, 0x8000);
    for (int i = 0; i < 100; i++) { sleep_ms(2); if (!(rmii_mac_mdio_read(s_phy, 0) & 0x8000)) break; }
    sleep_ms(10);
    rmii_mac_mdio_write(s_phy, 4, 0x0181);     // 100FD + 100HD + 802.3
    rmii_mac_mdio_write(s_phy, 0, 0x1200);     // auto-neg enable + restart
    LOGW("[MAC] auto-neg restarted (BMCR=0x%04X ANAR=0x%04X)\n",
         rmii_mac_mdio_read(s_phy, 0), rmii_mac_mdio_read(s_phy, 4));

    // ── PIO programs ──
    s_rx_off = pio_add_program(PIO_DEV, &lan8720_rx_program);
    s_tx_off = pio_add_program(PIO_DEV, &lan8720_tx_program);

    // ── RX DMA: PIO RX FIFO (high byte) → current rx buffer ──
    s_rx_dma = dma_claim_unused_channel(true);
    dma_channel_config rc = dma_channel_get_default_config(s_rx_dma);
    channel_config_set_read_increment(&rc, false);
    channel_config_set_write_increment(&rc, true);
    channel_config_set_dreq(&rc, pio_get_dreq(PIO_DEV, SM_RX, false));
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_8);
    dma_channel_set_config(s_rx_dma, &rc, false);
    dma_channel_set_read_addr(s_rx_dma, ((uint8_t *)&PIO_DEV->rxf[SM_RX]) + 3, false);
    for (int i = 0; i < RX_NBUF; i++) s_rxlen[i] = 0;
    dma_channel_set_write_addr(s_rx_dma, s_rxbuf[0], false);
    dma_channel_set_trans_count(s_rx_dma, RX_BUFSZ, true);

    // ── TX DMA: tx buffer → PIO TX FIFO (low byte) ──
    s_tx_dma = dma_claim_unused_channel(true);
    dma_channel_config tc = dma_channel_get_default_config(s_tx_dma);
    channel_config_set_read_increment(&tc, true);
    channel_config_set_write_increment(&tc, false);
    channel_config_set_dreq(&tc, pio_get_dreq(PIO_DEV, SM_TX, true));
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_8);
    dma_channel_set_config(s_tx_dma, &tc, false);
    dma_channel_set_write_addr(s_tx_dma, ((uint8_t *)&PIO_DEV->txf[SM_TX]) + 0, false);

    // Fixed preamble (7×0x55) + SFD (0xD5).
    for (int i = 0; i < 7; i++) s_txbuf[i] = 0x55;
    s_txbuf[7] = 0xD5;

    // ── Start the state machines ──
    lan8720_tx_init(PIO_DEV, SM_TX, s_tx_off, PICO_RMII_ETHERNET_TX_PIN);

    irq_set_exclusive_handler(PIO0_IRQ_0, rmii_rx_isr);
    pio_set_irq0_source_enabled(PIO_DEV, pis_interrupt0, true);
    irq_set_enabled(PIO0_IRQ_0, true);

    lan8720_rx_init(PIO_DEV, SM_RX, s_rx_off, PICO_RMII_ETHERNET_RX_PIN);

    LOGW("[MAC] PIO RMII MAC up (edge-synced to REF_CLK, clkdiv=1)\n");
    return true;
}
