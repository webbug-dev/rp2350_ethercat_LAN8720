// ethercat.c — a small EtherCAT master on the LAN8720 raw L2 MAC (src/rmii_mac.c),
// with a CoE/SDO mailbox layer for slave configuration.
//
// There is no EtherCAT slave-controller hardware here (no Distributed Clocks), so
// this is a soft, best-effort master suitable for a bench demo — not a
// hard-real-time fieldbus. What it does on the wire:
//   • sends/receives raw Ethernet frames (EtherType 0x88A4);
//   • discovers slaves with a Broadcast-Read of AL-Status (WKC == slave count);
//   • assigns each slave a station address (APWR 0x0010);
//   • reads the standard mailbox layout from each slave's SII (EEPROM) and sets
//     up the mailbox SyncManagers (SM0/SM1);
//   • walks the EtherCAT State Machine INIT → PRE-OP → SAFE-OP → OP, logging the
//     AL status (and AL status code on errors) at every step;
//   • over CoE in PRE-OP: reads identity (0x1018 vendor/product/revision/serial
//     and the 0x1008 device name) and configures the CiA-402 Cyclic-Sync-Velocity
//     PDO mapping (RxPDO 0x6040+0x60FF, TxPDO 0x6041+0x6064), then the process-data
//     SyncManagers (SM2/SM3) and FMMUs;
//   • runs the cyclic process-data loop (LWR outputs + LRD inputs) that streams
//     the button-driven velocity profile and reads back the actual position.
//
// SM/mailbox physical addresses default to the values in board.h (EC_MBX_*,
// EC_SM2/3_ADDR); the mailbox offsets/sizes are overridden from the SII when it
// reads cleanly. The process-data SM start addresses come from the ESI <Sm>
// elements — adjust board.h to match your A5 / LC10E if the bring-up logs show
// the slave refusing SAFE-OP/OP.

#include "ethercat.h"
#include "board.h"
#include "shared_state.h"
#include "rmii_mac.h"
#include "net_debug.h"
#include "log.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// ── EtherCAT command codes (datagram "cmd" byte) ────────────────────────────
#define EC_APRD   0x01   // auto-increment physical read
#define EC_APWR   0x02   // auto-increment physical write
#define EC_FPRD   0x04   // configured-address physical read
#define EC_FPWR   0x05   // configured-address physical write
#define EC_BRD    0x07   // broadcast read
#define EC_BWR    0x08   // broadcast write
#define EC_LRD    0x0A   // logical read
#define EC_LWR    0x0B   // logical write

// ── ESC (slave controller) registers ────────────────────────────────────────
#define REG_STATION    0x0010   // configured station address (RW)
#define REG_AL_CTRL    0x0120   // requested AL state (RW)
#define REG_AL_STATUS  0x0130   // actual AL state (RO)
#define REG_AL_CODE    0x0134   // AL status code (RO)
#define REG_EE_CONF    0x0500   // EEPROM configuration
#define REG_EE_CTRL    0x0502   // EEPROM control/status
#define REG_EE_ADDR    0x0504   // EEPROM address
#define REG_EE_DATA    0x0508   // EEPROM data
#define REG_SM0        0x0800   // SyncManager 0 (each SM is 8 bytes)
#define REG_SM1        0x0808
#define REG_SM2        0x0810
#define REG_SM3        0x0818
#define REG_SM1_STAT   0x080D   // SM1 status (mailbox-full in bit 3)
#define REG_FMMU0      0x0600   // FMMU 0 (each FMMU is 16 bytes)
#define REG_FMMU1      0x0610

// AL states (low nibble of AL-Control / AL-Status). Bit 4 = error.
#define AL_INIT   0x01
#define AL_PREOP  0x02
#define AL_SAFEOP 0x04
#define AL_OP     0x08
#define AL_ERR    0x10

// SII (EEPROM) standard mailbox-config word addresses.
#define SII_MBX_OUT_OFF  0x18    // RxMailbox offset (master→slave)
#define SII_MBX_OUT_SZ   0x19
#define SII_MBX_IN_OFF   0x1A    // TxMailbox offset (slave→master)
#define SII_MBX_IN_SZ    0x1B

// ── Logical process image (CiA-402 CSV) ─────────────────────────────────────
// Outputs: per drive { u16 controlword 0x6040; i32 target_velocity 0x60FF }.
// Inputs:  per drive { u16 statusword  0x6041; i32 position_actual 0x6064 }.
#define PD_OUT_PER_DRV   6
#define PD_IN_PER_DRV    6
#define PD_OUT_LEN       (EC_DRIVE_COUNT * PD_OUT_PER_DRV)
#define PD_IN_LEN        (EC_DRIVE_COUNT * PD_IN_PER_DRV)
#define PD_OUT_LOGICAL   0x00000000u
#define PD_IN_LOGICAL    0x00010000u

// CiA-402 controlword patterns.
#define CW_SHUTDOWN       0x0006
#define CW_ENABLE_OP      0x000F

#define EC_REPLY_TIMEOUT_MS  10

static uint8_t s_mac[6]     = { ETH_MAC0, ETH_MAC1, ETH_MAC2, ETH_MAC3, ETH_MAC4, ETH_MAC5 };
// EtherCAT slaves loop frames back unchanged; replies are matched by datagram
// index, not by MAC, so the source address is arbitrary (kept distinct from
// s_mac out of habit — last byte differs).
static uint8_t s_src_mac[6] = { ETH_MAC0, ETH_MAC1, ETH_MAC2, ETH_MAC3, ETH_MAC4, (uint8_t)(ETH_MAC5 ^ 0x01) };
static uint8_t s_idx;            // rolling datagram index (reply matching)
static uint8_t s_tx[256];
static uint8_t s_rx[256];
static bool    s_dbg;            // verbose per-frame trace (set during probes)
static bool    s_coe_quiet;      // suppress SDO error logs (optional-object dump)

typedef struct {
    bool     present;
    int      drive_idx;          // EC_DRV_SERVO / EC_DRV_STEP
    uint16_t station;            // configured station address
    uint16_t mbx_out, mbx_out_sz;
    uint16_t mbx_in,  mbx_in_sz;
    uint8_t  mbx_cnt;            // CoE mailbox counter (1..7)
    uint32_t vendor, product, revision, serial;
    char     name[20];
    bool     op;                 // reached OPERATIONAL
} ec_slave_t;
static ec_slave_t s_slave[EC_DRIVE_COUNT];

// ── little-endian byte helpers ───────────────────────────────────────────────
static inline uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t rd32s(const uint8_t *p) { return (int32_t)rd32(p); }
static inline void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void wr32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

static const char *al_name(uint8_t st) {
    switch (st & 0x0F) {
        case AL_INIT: return "INIT"; case AL_PREOP: return "PRE-OP";
        case AL_SAFEOP: return "SAFE-OP"; case AL_OP: return "OP"; default: return "?";
    }
}

// ── LAN8720 RMII raw-frame I/O (via the PIO MAC) ─────────────────────────────
static bool ec_link_up(void) { return rmii_mac_link_up(); }

// Send the frame built in s_tx (len bytes, no FCS). The MAC adds preamble + FCS.
static bool ec_send_raw(uint16_t len) {
    rmii_mac_send(s_tx, len);
    if (s_dbg) LOGW("[EC] TX %u bytes\n", len);
    return true;
}

// Hex-dump a buffer to the log (verbose probes only).
static void hexdump(const char *tag, const uint8_t *p, int n) {
    char b[100]; int o = 0;
    for (int i = 0; i < n && o < 92; i++) o += snprintf(b + o, sizeof(b) - o, "%02X ", p[i]);
    LOGW("[EC] %s[%d]: %s\n", tag, n, b);
}

// Poll the MAC for the next received frame (FCS already stripped) into s_rx.
static uint16_t ec_recv_raw(uint32_t timeout_ms) {
    uint32_t start = now_ms();
    for (;;) {
        int n = rmii_mac_recv(s_rx, sizeof(s_rx));
        if (n > 0) return (uint16_t)n;
        if (now_ms() - start >= timeout_ms) return 0;
    }
}

static void put_eth_header(void) {
    memset(s_tx, 0xFF, 6);              // dst = broadcast
    memcpy(s_tx + 6, s_src_mac, 6);     // src (arbitrary; see s_src_mac note)
    s_tx[12] = (uint8_t)(EC_ETHERTYPE >> 8);
    s_tx[13] = (uint8_t)(EC_ETHERTYPE & 0xFF);
}

// Single-datagram command (BRD/BWR/APRD/APWR/FPRD/FPWR). For read commands the
// returned value is copied back into `data`. Returns the working counter.
static uint16_t ec_cmd(uint8_t cmd, uint16_t adp, uint16_t ado, uint8_t *data, uint16_t len) {
    uint8_t idx = ++s_idx;
    put_eth_header();
    uint16_t dgram = (uint16_t)(10 + len + 2);
    wr16(s_tx + 14, (uint16_t)((dgram & 0x7FF) | (1u << 12)));
    s_tx[16] = cmd;
    s_tx[17] = idx;
    wr16(s_tx + 18, adp);
    wr16(s_tx + 20, ado);
    wr16(s_tx + 22, (uint16_t)(len & 0x7FF));
    wr16(s_tx + 24, 0);
    if (data && len) memcpy(s_tx + 26, data, len); else memset(s_tx + 26, 0, len);
    wr16(s_tx + 26 + len, 0);
    uint16_t total = (uint16_t)(28 + len);
    if (total < 60) { memset(s_tx + total, 0, 60 - total); total = 60; }
    if (s_dbg) LOGW("[EC] TX cmd=0x%02X ado=0x%04X len=%u frame=%uB\n",
                    cmd, ado, len, total);
    ec_send_raw(total);

    for (int tries = 0; tries < 4; tries++) {
        uint16_t n = ec_recv_raw(EC_REPLY_TIMEOUT_MS);
        if (s_dbg && (n || tries == 0)) LOGW("[EC] rx try %d: n=%u\n", tries, n);
        if (!n) break;
        if (s_dbg) hexdump("rx", s_rx, n < 24 ? n : 24);
        if (n < (uint16_t)(28 + len)) continue;
        if (s_rx[12] != (uint8_t)(EC_ETHERTYPE >> 8) || s_rx[13] != (uint8_t)(EC_ETHERTYPE & 0xFF)) continue;
        if (s_rx[17] != idx) continue;
        if (data && len) memcpy(data, s_rx + 26, len);
        uint16_t wkc = rd16(s_rx + 26 + len);
        if (s_dbg) LOGW("[EC] reply OK idx=%u WKC=%u\n", idx, wkc);
        return wkc;
    }
    return 0;
}

// Addressing wrappers.
static inline uint16_t autoinc(int pos) { return (uint16_t)(0 - pos); }
static uint16_t ec_apwr(int pos, uint16_t ado, uint8_t *d, uint16_t len) { return ec_cmd(EC_APWR, autoinc(pos), ado, d, len); }
static uint16_t ec_aprd(int pos, uint16_t ado, uint8_t *d, uint16_t len) { return ec_cmd(EC_APRD, autoinc(pos), ado, d, len); }
static uint16_t ec_fpwr(uint16_t sta, uint16_t ado, uint8_t *d, uint16_t len) { return ec_cmd(EC_FPWR, sta, ado, d, len); }
static uint16_t ec_fprd(uint16_t sta, uint16_t ado, uint8_t *d, uint16_t len) { return ec_cmd(EC_FPRD, sta, ado, d, len); }

// ── SII (EEPROM) word read ───────────────────────────────────────────────────
static bool sii_read(uint16_t sta, uint16_t word_addr, uint16_t *out) {
    uint8_t conf = 0x00; ec_fpwr(sta, REG_EE_CONF, &conf, 1);   // assign EEPROM to master
    uint8_t a[4] = {0}; wr32(a, word_addr); ec_fpwr(sta, REG_EE_ADDR, a, 4);
    uint8_t cmd[2]; wr16(cmd, 0x0100); ec_fpwr(sta, REG_EE_CTRL, cmd, 2);   // read command
    for (int i = 0; i < 200; i++) {
        uint8_t st[2] = {0,0};
        if (ec_fprd(sta, REG_EE_CTRL, st, 2) == 0) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
        uint16_t v = rd16(st);
        if (!(v & 0x8000)) {                     // not busy
            if (v & 0x6800) return false;          // ack/checksum/command error
            uint8_t data[8] = {0};
            if (ec_fprd(sta, REG_EE_DATA, data, 8) == 0) return false;
            *out = rd16(data);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// ── CoE / SDO mailbox ────────────────────────────────────────────────────────
static bool mbx_wait_full(ec_slave_t *sl) {
    for (int i = 0; i < 200; i++) {
        uint8_t st = 0;
        if (ec_fprd(sl->station, REG_SM1_STAT, &st, 1) && (st & 0x08)) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

// Build the 6-byte mailbox header + CoE header into `m`, returns offset of the
// SDO command byte (8). `sdo_len` is the SDO payload length (8 for expedited).
static uint8_t mbx_coe_header(ec_slave_t *sl, uint8_t *m, uint16_t sdo_len) {
    sl->mbx_cnt = (uint8_t)((sl->mbx_cnt % 7) + 1);     // 1..7, never 0
    wr16(m + 0, (uint16_t)(2 + sdo_len));               // mailbox length (CoE hdr + SDO)
    wr16(m + 2, 0);                                     // address
    m[4] = 0;                                           // channel + priority
    m[5] = (uint8_t)((sl->mbx_cnt << 4) | 0x03);        // counter + type=CoE(3)
    wr16(m + 6, 0x2000);                                // CoE header: SDO request (svc 2)
    return 8;
}

// A mailbox SyncManager only latches when the master writes through to the LAST
// byte of the SM buffer — so every mailbox message is zero-padded to the full SM
// length (mbx_out_sz) and written in one FPWR. Shared buffer for send + reply.
static uint8_t s_mbx[256];

static bool mbx_xfer(ec_slave_t *sl, const char *what, uint16_t index, uint8_t sub) {
    uint16_t wl = sl->mbx_out_sz; if (wl > sizeof(s_mbx)) wl = sizeof(s_mbx);
    if (ec_fpwr(sl->station, sl->mbx_out, s_mbx, wl) == 0) {
        if (!s_coe_quiet) LOGE("[EC] %s 0x%04X:%u FPWR WKC=0\n", what, index, sub); return false; }
    if (!mbx_wait_full(sl)) {
        if (!s_coe_quiet) LOGE("[EC] %s 0x%04X:%u no mbx response\n", what, index, sub); return false; }
    memset(s_mbx, 0, sizeof(s_mbx));
    uint16_t rl = sl->mbx_in_sz; if (rl > sizeof(s_mbx)) rl = sizeof(s_mbx);
    if (ec_fprd(sl->station, sl->mbx_in, s_mbx, rl) == 0) {
        if (!s_coe_quiet) LOGE("[EC] %s 0x%04X:%u mbx read WKC=0\n", what, index, sub); return false; }
    return true;
}

// SDO upload (read object). Returns bytes read into `out` (≤cap), or -1.
static int coe_upload(ec_slave_t *sl, uint16_t index, uint8_t sub, uint8_t *out, int cap) {
    memset(s_mbx, 0, sizeof(s_mbx));
    uint8_t o = mbx_coe_header(sl, s_mbx, 8);
    s_mbx[o] = 0x40;                                    // SDO upload request
    wr16(s_mbx + o + 1, index); s_mbx[o + 3] = sub;
    if (!mbx_xfer(sl, "SDO up", index, sub)) return -1;
    uint8_t cmd = s_mbx[8];
    if (cmd & 0x80) { if (!s_coe_quiet) LOGE("[EC] SDO up 0x%04X:%u ABORT 0x%08lX\n", index, sub, (unsigned long)rd32(s_mbx + 12)); return -1; }
    if ((cmd >> 5) != 0x02) { if (!s_coe_quiet) LOGE("[EC] SDO up 0x%04X:%u bad resp 0x%02X\n", index, sub, cmd); return -1; }
    int dl;
    const uint8_t *src;
    if (cmd & 0x02) { dl = 4 - ((cmd >> 2) & 0x03); src = s_mbx + 12; }  // expedited
    else            { dl = (int)rd32(s_mbx + 12);    src = s_mbx + 16; }  // normal
    if (dl > cap) dl = cap;
    if (dl > 0) memcpy(out, src, dl);
    return dl;
}

// SDO download (write object, expedited, 1/2/4 bytes). Returns true on confirm.
static bool coe_download(ec_slave_t *sl, uint16_t index, uint8_t sub, uint32_t val, int len) {
    memset(s_mbx, 0, sizeof(s_mbx));
    uint8_t o = mbx_coe_header(sl, s_mbx, 8);
    s_mbx[o] = (uint8_t)(0x23 | ((4 - len) << 2));      // expedited download, size set
    wr16(s_mbx + o + 1, index); s_mbx[o + 3] = sub;
    wr32(s_mbx + o + 4, val);
    if (!mbx_xfer(sl, "SDO dn", index, sub)) return false;
    if (s_mbx[8] & 0x80) { LOGE("[EC] SDO dn 0x%04X:%u ABORT 0x%08lX\n", index, sub, (unsigned long)rd32(s_mbx + 12)); return false; }
    return ((s_mbx[8] >> 5) == 0x03);                   // download response
}

// CoE SDO-Information service (object-dictionary enumeration). Sends one request
// (opcode + payload) and concatenates the possibly-fragmented response's service
// data into `resp`. Returns bytes collected, or -1. CoE service id = 0x08.
static int ec_sdo_info(ec_slave_t *sl, uint8_t opcode, const uint8_t *req, int reqlen,
                       uint8_t *resp, int cap) {
    memset(s_mbx, 0, sizeof(s_mbx));
    sl->mbx_cnt = (uint8_t)((sl->mbx_cnt % 7) + 1);
    uint16_t sdolen = (uint16_t)(4 + reqlen);            // opcode + incomplete + fragleft(2) + payload
    wr16(s_mbx + 0, (uint16_t)(2 + sdolen));             // mbx length = CoE(2) + sdo-info
    wr16(s_mbx + 2, 0); s_mbx[4] = 0;
    s_mbx[5] = (uint8_t)((sl->mbx_cnt << 4) | 0x03);     // CoE
    wr16(s_mbx + 6, 0x8000);                             // CoE header: SDO-Information (svc 8)
    s_mbx[8] = opcode; s_mbx[9] = 0; wr16(s_mbx + 10, 0);
    if (reqlen > 0) memcpy(s_mbx + 12, req, reqlen);

    uint16_t wl = sl->mbx_out_sz; if (wl > sizeof(s_mbx)) wl = sizeof(s_mbx);
    if (ec_fpwr(sl->station, sl->mbx_out, s_mbx, wl) == 0) return -1;

    int total = 0;
    for (int frag = 0; frag < 64; frag++) {
        if (!mbx_wait_full(sl)) return total > 0 ? total : -1;
        memset(s_mbx, 0, sizeof(s_mbx));
        uint16_t rl = sl->mbx_in_sz; if (rl > sizeof(s_mbx)) rl = sizeof(s_mbx);
        if (ec_fprd(sl->station, sl->mbx_in, s_mbx, rl) == 0) return total > 0 ? total : -1;
        if (s_mbx[8] == 0x07) return -1;                 // SDO-info error response
        bool incomplete = (s_mbx[9] & 0x80) != 0;
        int dlen = (int)rd16(s_mbx) - 2 - 4;             // service data this fragment
        if (dlen < 0) dlen = 0;
        if (total + dlen > cap) dlen = cap - total;
        if (dlen > 0) { memcpy(resp + total, s_mbx + 12, dlen); total += dlen; }
        if (!incomplete) break;
    }
    return total;
}

// ── AL state machine ─────────────────────────────────────────────────────────
static bool ec_set_state(ec_slave_t *sl, uint8_t state) {
    uint8_t c[2]; wr16(c, state); ec_fpwr(sl->station, REG_AL_CTRL, c, 2);
    for (int i = 0; i < 250; i++) {
        uint8_t s[2] = {0,0};
        if (ec_fprd(sl->station, REG_AL_STATUS, s, 2)) {
            uint16_t als = rd16(s);
            if (als & AL_ERR) {
                uint8_t code[2] = {0,0}; ec_fprd(sl->station, REG_AL_CODE, code, 2);
                LOGE("[EC] sta 0x%04X -> %s REFUSED, AL=0x%02X code=0x%04X\n",
                    sl->station, al_name(state), als, rd16(code));
                uint8_t ack[2]; wr16(ack, (uint16_t)(state | AL_ERR)); ec_fpwr(sl->station, REG_AL_CTRL, ack, 2);
                return false;
            }
            if ((als & 0x0F) == (state & 0x0F)) return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    LOGE("[EC] sta 0x%04X -> %s TIMEOUT\n", sl->station, al_name(state));
    return false;
}

// ── SyncManager / FMMU configuration ─────────────────────────────────────────
static void sm_config(uint16_t sta, uint16_t reg, uint16_t phys, uint16_t len, uint8_t ctrl) {
    uint8_t sm[8] = {0};
    wr16(sm + 0, phys); wr16(sm + 2, len); sm[4] = ctrl; sm[5] = 0; sm[6] = 0x01; sm[7] = 0x00;
    ec_fpwr(sta, reg, sm, 8);
}
static void fmmu_config(uint16_t sta, uint16_t reg, uint32_t log, uint16_t len, uint16_t phys, uint8_t type) {
    uint8_t f[16] = {0};
    wr32(f + 0, log); wr16(f + 4, len); f[6] = 0x00; f[7] = 0x07;
    wr16(f + 8, phys); f[10] = 0x00; f[11] = type; f[12] = 0x01;
    ec_fpwr(sta, reg, f, 16);
}

// Configure the CiA-402 Cyclic-Sync-Velocity PDO mapping over CoE (best effort —
// some drives ship a fixed mapping and reject these; we log and carry on).
static void coe_map_csp(ec_slave_t *sl) {
    bool ok = true;
    ok &= coe_download(sl, 0x1C12, 0, 0, 1);             // clear RxPDO assign
    ok &= coe_download(sl, 0x1600, 0, 0, 1);             // clear RxPDO mapping
    ok &= coe_download(sl, 0x1600, 1, 0x60400010, 4);    // controlword    (0x6040:00, 16b)
    ok &= coe_download(sl, 0x1600, 2, 0x607A0020, 4);    // target position(0x607A:00, 32b)
    ok &= coe_download(sl, 0x1600, 0, 2, 1);
    ok &= coe_download(sl, 0x1C12, 1, 0x1600, 2);
    ok &= coe_download(sl, 0x1C12, 0, 1, 1);
    ok &= coe_download(sl, 0x1C13, 0, 0, 1);             // clear TxPDO assign
    ok &= coe_download(sl, 0x1A00, 0, 0, 1);
    ok &= coe_download(sl, 0x1A00, 1, 0x60410010, 4);    // statusword     (0x6041:00, 16b)
    ok &= coe_download(sl, 0x1A00, 2, 0x60640020, 4);    // position actual(0x6064:00, 32b)
    ok &= coe_download(sl, 0x1A00, 0, 2, 1);
    ok &= coe_download(sl, 0x1C13, 1, 0x1A00, 2);
    ok &= coe_download(sl, 0x1C13, 0, 1, 1);
    coe_download(sl, 0x6060, 0, 8, 1);                   // modes of operation = CSP(8)
    LOGW("[EC] sta 0x%04X PDO map (CSP) %s\n", sl->station, ok ? "configured" : "partial (fixed-mapping drive?)");
}

// ── shared-state helper ──────────────────────────────────────────────────────
static void set_phase(int d, drive_phase_t ph, const char *msg) {
    SHARED_LOCK();
    g_shared.drive[d].phase = ph;
    if (msg) { strncpy(g_shared.drive[d].msg, msg, sizeof(g_shared.drive[d].msg) - 1);
               g_shared.drive[d].msg[sizeof(g_shared.drive[d].msg) - 1] = '\0'; }
    SHARED_UNLOCK();
}

// ── CoE decode tables ────────────────────────────────────────────────────────
static const char *ec_vendor_name(uint32_t id) {
    switch (id) {
        case 0x0A79: return "Lichuan (Shenzhen LC Electric)";
        case 0x0002: return "Beckhoff";
        case 0x0626: return "Trinamic / ADI";
        default:     return "unknown vendor";
    }
}
static const char *ec_model_name(uint32_t vendor, uint32_t product) {
    if (vendor == 0x0A79) switch (product) {       // from doc/ ESI files
        case 0x1000: return "CL3-E57H";
        case 0x1100: return "CL3-E86H";
        case 0x1200: return "OL3-E57H";
        case 0x2000: return "OL3-E86H";
        case 0x0402: return "LC10E";
    }
    return "unknown model";
}
static const char *ec_mode_name(int m) {
    switch (m) { case 0: return "none"; case 1: return "pp"; case 2: return "vl";
        case 3: return "pv"; case 4: return "tq"; case 6: return "hm"; case 7: return "ip";
        case 8: return "csp"; case 9: return "csv"; case 10: return "cst"; default: return "?"; }
}
// CiA-402 supported-drive-modes bitmask (0x6502) → "pp pv hm csp"
static void ec_modes_str(uint32_t mask, char *b, size_t cap) {
    static const struct { uint32_t bit; const char *n; } M[] = {
        {1u<<0,"pp"}, {1u<<1,"vl"}, {1u<<2,"pv"}, {1u<<3,"tq"}, {1u<<5,"hm"},
        {1u<<6,"ip"}, {1u<<7,"csp"}, {1u<<8,"csv"}, {1u<<9,"cst"},
    };
    size_t o = 0; b[0] = '\0';
    for (unsigned i = 0; i < sizeof(M)/sizeof(M[0]); i++)
        if (mask & M[i].bit) o += (size_t)snprintf(b + o, cap - o, "%s%s", o ? " " : "", M[i].n);
    if (!o) snprintf(b, cap, "none");
}

// ── CoE object-dictionary dump (read max identity/characteristics via SDO) ───
static bool coe_get_uint(ec_slave_t *sl, uint16_t idx, uint8_t sub, uint32_t *out) {
    uint8_t b[4] = {0};
    int n = coe_upload(sl, idx, sub, b, 4);
    if (n <= 0) return false;
    uint32_t v = 0;
    for (int i = 0; i < n && i < 4; i++) v |= (uint32_t)b[i] << (8 * i);
    *out = v; return true;
}
static void coe_log_str(ec_slave_t *sl, uint16_t idx, uint8_t sub, const char *name) {
    char b[40];
    int n = coe_upload(sl, idx, sub, (uint8_t *)b, (int)sizeof(b) - 1);
    if (n > 0) { b[n] = '\0'; LOGW("[EC]   %-16s = '%s'\n", name, b); }
}
static void coe_log_uint(ec_slave_t *sl, uint16_t idx, uint8_t sub, const char *name) {
    uint8_t b[4] = {0};
    int n = coe_upload(sl, idx, sub, b, 4);
    if (n <= 0) return;
    uint32_t v = 0;
    for (int i = 0; i < n && i < 4; i++) v |= (uint32_t)b[i] << (8 * i);
    LOGW("[EC]   %-16s = %lu (0x%lX)\n", name, (unsigned long)v, (unsigned long)v);
}

// Read and log a curated set of standard CiA-301/402 objects (model, vendor,
// versions, motor + encoder characteristics). Unsupported objects abort quietly
// and are skipped. This is the practical equivalent of SOEM's slaveinfo for the
// well-known objects (full OD enumeration would need the SDO-Info service).
static const char *ec_objcode(uint8_t c) {
    switch (c) { case 7: return "VAR"; case 8: return "ARR"; case 9: return "REC"; default: return "?"; }
}
static const char *ec_dtype(uint16_t t) {
    switch (t) {
        case 0x0001: return "BOOL"; case 0x0002: return "I8";  case 0x0003: return "I16";
        case 0x0004: return "I32";  case 0x0005: return "U8";  case 0x0006: return "U16";
        case 0x0007: return "U32";  case 0x0008: return "R32"; case 0x0009: return "STR";
        case 0x000A: return "OSTR"; case 0x0011: return "R64"; case 0x0015: return "I64";
        case 0x001B: return "U64";  default: return "?";
    }
}

// Full object-dictionary enumeration via SDO-Information: list every object, then
// read each object's description (name/code/#subs) and each entry's description
// (name/type/bits/access). Logged once per boot — it's slow + verbose.
static void coe_dump_od(ec_slave_t *sl) {
    static uint8_t list[512];
    uint8_t lt[2]; wr16(lt, 0x0001);                     // list type 1 = all objects
    int n = ec_sdo_info(sl, 0x01, lt, 2, list, sizeof(list));
    if (n < 4) { LOGW("[EC]   (SDO-Information not supported by this drive)\n"); return; }
    int cnt = (n - 2) / 2;
    LOGW("[EC] --- full OD: %d objects ---\n", cnt);
    for (int i = 0; i < cnt; i++) {
        uint16_t req = rd16(list + 2 + 2 * i);
        uint8_t rq[2]; wr16(rq, req);
        uint8_t ob[128];
        int m = ec_sdo_info(sl, 0x03, rq, 2, ob, sizeof(ob));
        if (m < 6) continue;
        uint16_t idx = rd16(ob + 0);                     // index echoed by the slave
        uint16_t dt = rd16(ob + 2); uint8_t maxsub = ob[4]; uint8_t code = ob[5];
        int nl = m - 6; if (nl > 40) nl = 40; if (nl < 0) nl = 0;
        char nm[44]; memcpy(nm, ob + 6, nl); nm[nl] = '\0';
        LOGW("[EC]   0x%04X %-3s subs=%u type=%s  %s\n", idx, ec_objcode(code), maxsub, ec_dtype(dt), nm);
    }
    LOGW("[EC] --- end OD ---\n");
}

static void coe_dump_info(ec_slave_t *sl) {
    LOGW("[EC] === sta 0x%04X CoE info dump ===\n", sl->station);
    s_coe_quiet = true;
    uint32_t v;
    if (coe_get_uint(sl, 0x1000, 0, &v))
        LOGW("[EC]   Device type      = 0x%lX  profile %lu%s\n", (unsigned long)v,
             (unsigned long)(v & 0xFFFF), (v & 0xFFFF) == 402 ? " (CiA-402 drive)" : "");
    coe_log_str (sl, 0x1008, 0, "Device name");
    coe_log_str (sl, 0x1009, 0, "Hardware ver");
    coe_log_str (sl, 0x100A, 0, "Software ver");
    LOGW("[EC]   Vendor ID        = 0x%lX  %s\n", (unsigned long)sl->vendor, ec_vendor_name(sl->vendor));
    LOGW("[EC]   Product code     = 0x%lX  %s\n", (unsigned long)sl->product, ec_model_name(sl->vendor, sl->product));
    LOGW("[EC]   Revision/Serial  = 0x%lX / 0x%lX\n", (unsigned long)sl->revision, (unsigned long)sl->serial);
    if (coe_get_uint(sl, 0x6502, 0, &v)) {
        char m[48]; ec_modes_str(v, m, sizeof m);
        LOGW("[EC]   Supported modes  = 0x%lX  [%s]\n", (unsigned long)v, m);
    }
    if (coe_get_uint(sl, 0x6061, 0, &v))
        LOGW("[EC]   Mode display     = %lu  %s\n", (unsigned long)v, ec_mode_name((int)v));
    coe_log_str (sl, 0x6403, 0, "Motor catalog");
    coe_log_str (sl, 0x6404, 0, "Motor mfr");
    coe_log_uint(sl, 0x6402, 0, "Motor type");
    coe_log_uint(sl, 0x6075, 0, "Rated current mA");
    coe_log_uint(sl, 0x6076, 0, "Rated torque");
    coe_log_uint(sl, 0x6080, 0, "Max motor speed");
    coe_log_uint(sl, 0x608F, 1, "Enc increments");
    coe_log_uint(sl, 0x608F, 2, "Enc motor revs");
    coe_log_uint(sl, 0x6091, 1, "Gear motor revs");
    coe_log_uint(sl, 0x6091, 2, "Gear shaft revs");
    coe_log_uint(sl, 0x603F, 0, "Error code");

    // Full OD enumeration (SDO-Information) — once per boot (slow + verbose).
    static bool s_od_done = false;
    if (!s_od_done) { s_od_done = true; coe_dump_od(sl); }

    s_coe_quiet = false;
    LOGW("[EC] === end dump ===\n");
}

// ── Bring one slave (at auto-increment position `pos`) up to OP ───────────────
static bool bring_up(int pos) {
    ec_slave_t *sl = &s_slave[pos];
    memset(sl, 0, sizeof(*sl));
    sl->station  = (uint16_t)(EC_STATION_BASE + pos);
    sl->drive_idx = pos;                                 // refined by product code below

    // (1) Read AL state by auto-increment address, before we assign a station.
    uint8_t als[2] = {0,0};
    uint16_t wkc = ec_aprd(pos, REG_AL_STATUS, als, 2);
    LOGW("[EC] pos %d: present (APRD WKC=%u) AL=%s(0x%02X)\n", pos, wkc, al_name(als[0]), als[0]);

    // (2) Assign a configured station address.
    uint8_t sa[2]; wr16(sa, sl->station);
    wkc = ec_apwr(pos, REG_STATION, sa, 2);
    LOGW("[EC] pos %d: station <- 0x%04X (APWR WKC=%u)\n", pos, sl->station, wkc);

    // (3) INIT, then read the mailbox layout from the SII (fallback to board.h).
    ec_set_state(sl, AL_INIT);
    uint16_t v;
    sl->mbx_out = EC_MBX_OUT_ADDR; sl->mbx_out_sz = EC_MBX_SIZE;
    sl->mbx_in  = EC_MBX_IN_ADDR;  sl->mbx_in_sz  = EC_MBX_SIZE;
    bool sii_ok = true;
    if (sii_read(sl->station, SII_MBX_OUT_OFF, &v) && v) sl->mbx_out = v;     else sii_ok = false;
    if (sii_read(sl->station, SII_MBX_OUT_SZ,  &v) && v) sl->mbx_out_sz = v;  else sii_ok = false;
    if (sii_read(sl->station, SII_MBX_IN_OFF,  &v) && v) sl->mbx_in = v;      else sii_ok = false;
    if (sii_read(sl->station, SII_MBX_IN_SZ,   &v) && v) sl->mbx_in_sz = v;   else sii_ok = false;
    LOGW("[EC] sta 0x%04X mailbox %s: out @0x%04X/%uB in @0x%04X/%uB\n",
        sl->station, sii_ok ? "(SII)" : "(board.h fallback)",
        sl->mbx_out, sl->mbx_out_sz, sl->mbx_in, sl->mbx_in_sz);

    // (4) Configure the mailbox SyncManagers and go PRE-OP (mailbox active there).
    sm_config(sl->station, REG_SM0, sl->mbx_out, sl->mbx_out_sz, 0x26);   // mbx write
    sm_config(sl->station, REG_SM1, sl->mbx_in,  sl->mbx_in_sz,  0x22);   // mbx read
    if (!ec_set_state(sl, AL_PREOP)) { LOGE("[EC] sta 0x%04X stuck below PRE-OP\n", sl->station); return false; }
    LOGW("[EC] sta 0x%04X -> PRE-OP ok (mailbox up)\n", sl->station);

    // (5) Read identity over CoE.
    uint8_t buf[20];
    if (coe_upload(sl, 0x1018, 1, buf, 4) == 4) sl->vendor   = rd32(buf);
    if (coe_upload(sl, 0x1018, 2, buf, 4) == 4) sl->product  = rd32(buf);
    if (coe_upload(sl, 0x1018, 3, buf, 4) == 4) sl->revision = rd32(buf);
    if (coe_upload(sl, 0x1018, 4, buf, 4) == 4) sl->serial   = rd32(buf);
    int nl = coe_upload(sl, 0x1008, 0, (uint8_t *)sl->name, (int)sizeof(sl->name) - 1);
    if (nl > 0) sl->name[nl] = '\0'; else strcpy(sl->name, "?");
    LOGW("[EC] sta 0x%04X identity: Vendor=0x%08lX Product=0x%08lX Rev=0x%08lX Ser=0x%08lX '%s'\n",
        sl->station, (unsigned long)sl->vendor, (unsigned long)sl->product,
        (unsigned long)sl->revision, (unsigned long)sl->serial, sl->name);

    // Decide which drive this is (product-code match if configured, else order).
    if (EC_SERVO_PRODUCT && sl->product == EC_SERVO_PRODUCT)      sl->drive_idx = EC_DRV_SERVO;
    else if (EC_STEP_PRODUCT && sl->product == EC_STEP_PRODUCT)   sl->drive_idx = EC_DRV_STEP;
    LOGW("[EC] sta 0x%04X => %s\n", sl->station,
        sl->drive_idx == EC_DRV_SERVO ? "ServoDrive" : "StepDrive");

    // Publish the decoded model name for the UI.
    SHARED_LOCK();
    strncpy(g_shared.drive[sl->drive_idx].model, ec_model_name(sl->vendor, sl->product),
            sizeof(g_shared.drive[sl->drive_idx].model) - 1);
    g_shared.drive[sl->drive_idx].model[sizeof(g_shared.drive[sl->drive_idx].model) - 1] = '\0';
    SHARED_UNLOCK();

    // (5b) Dump as much standard CoE info as the drive exposes.
    coe_dump_info(sl);

    // (6) PDO mapping + process-data SyncManagers + FMMUs.
    coe_map_csp(sl);
    sm_config(sl->station, REG_SM2, EC_SM2_ADDR, PD_OUT_PER_DRV, 0x64);   // outputs
    sm_config(sl->station, REG_SM3, EC_SM3_ADDR, PD_IN_PER_DRV,  0x20);   // inputs
    fmmu_config(sl->station, REG_FMMU0, PD_OUT_LOGICAL + pos * PD_OUT_PER_DRV,
                PD_OUT_PER_DRV, EC_SM2_ADDR, 0x02);                       // logical → outputs
    fmmu_config(sl->station, REG_FMMU1, PD_IN_LOGICAL + pos * PD_IN_PER_DRV,
                PD_IN_PER_DRV, EC_SM3_ADDR, 0x01);                        // inputs → logical

    // (7) SAFE-OP then OP.
    if (!ec_set_state(sl, AL_SAFEOP)) { LOGE("[EC] sta 0x%04X stuck below SAFE-OP\n", sl->station); return false; }
    LOGW("[EC] sta 0x%04X -> SAFE-OP ok\n", sl->station);
    sl->op = ec_set_state(sl, AL_OP);
    LOGW("[EC] sta 0x%04X -> OP %s\n", sl->station, sl->op ? "ok" : "PENDING (needs cyclic data)");
    sl->present = true;
    return true;
}

// ── Discovery + bring-up of the segment ──────────────────────────────────────
static int s_npos;               // slave positions present on the bus

static bool scan_and_bringup(void) {
    LOGW("[EC] link %s (%d Mbit) — scanning segment\n",
        rmii_mac_link_up() ? "UP" : "DOWN", rmii_mac_link_speed());

    uint8_t st[2] = {0,0};
    s_dbg = true;                                  // trace this BRD fully
    int n = (int)ec_cmd(EC_BRD, 0x0000, REG_AL_STATUS, st, 2);
    s_dbg = false;
    LOGW("[EC] === segment scan: BRD WKC=%d (slaves answering), AL=0x%02X ===\n", n, st[0]);

    if (n <= 0) {
        for (int d = 0; d < EC_DRIVE_COUNT; d++) set_phase(d, DS_SCAN, "Searching...");
        return false;
    }
    if (n > EC_DRIVE_COUNT) { LOGW("[EC] %d slaves seen, only %d supported\n", n, EC_DRIVE_COUNT); n = EC_DRIVE_COUNT; }
    s_npos = n;

    bool got[EC_DRIVE_COUNT] = { false };
    int up = 0;
    for (int pos = 0; pos < n; pos++) {
        if (!bring_up(pos)) { set_phase(s_slave[pos].drive_idx, DS_FAULT, "Bring-up failed"); continue; }
        got[s_slave[pos].drive_idx] = true;
        set_phase(s_slave[pos].drive_idx, DS_CONNECTED, "Connected");
        up++;
    }
    for (int d = 0; d < EC_DRIVE_COUNT; d++)
        if (!got[d]) { set_phase(d, DS_SCAN, "Searching...");
                       LOGW("[EC] %s not on the bus\n", d == EC_DRV_SERVO ? "ServoDrive" : "StepDrive"); }

    if (up == 0) return false;
    LOGW("[EC] === %d drive(s) configured — entering cyclic operation ===\n", up);
    return true;
}

// ── Cyclic process data: LWR (outputs) + LRD (inputs) in one frame ───────────
static uint16_t ec_exchange(const uint8_t *out, uint8_t *in) {
    uint8_t idx0 = ++s_idx, idx1 = ++s_idx;
    put_eth_header();
    uint16_t dg0 = (uint16_t)(10 + PD_OUT_LEN + 2);
    uint16_t dg1 = (uint16_t)(10 + PD_IN_LEN + 2);
    wr16(s_tx + 14, (uint16_t)(((dg0 + dg1) & 0x7FF) | (1u << 12)));

    uint8_t *d0 = s_tx + 16;
    d0[0] = EC_LWR; d0[1] = idx0;
    wr32(d0 + 2, PD_OUT_LOGICAL);
    wr16(d0 + 6, (uint16_t)((PD_OUT_LEN & 0x7FF) | (1u << 15)));   // M=1
    wr16(d0 + 8, 0);
    memcpy(d0 + 10, out, PD_OUT_LEN);
    wr16(d0 + 10 + PD_OUT_LEN, 0);

    uint8_t *d1 = s_tx + 16 + dg0;
    d1[0] = EC_LRD; d1[1] = idx1;
    wr32(d1 + 2, PD_IN_LOGICAL);
    wr16(d1 + 6, (uint16_t)(PD_IN_LEN & 0x7FF));                   // M=0
    wr16(d1 + 8, 0);
    memset(d1 + 10, 0, PD_IN_LEN);
    wr16(d1 + 10 + PD_IN_LEN, 0);

    uint16_t total = (uint16_t)(16 + dg0 + dg1);
    if (total < 60) { memset(s_tx + total, 0, 60 - total); total = 60; }
    ec_send_raw(total);

    for (int tries = 0; tries < 4; tries++) {
        uint16_t n = ec_recv_raw(EC_REPLY_TIMEOUT_MS);
        if (!n) break;
        if (n < (uint16_t)(16 + dg0 + dg1)) continue;
        if (s_rx[17] != idx0) continue;
        const uint8_t *r1 = s_rx + 16 + dg0;
        memcpy(in, r1 + 10, PD_IN_LEN);
        return rd16(r1 + 10 + PD_IN_LEN);
    }
    return 0;
}

// ── Velocity profile (one per drive): hold-to-run with a smooth stop ─────────
typedef struct { float t_ms; float rpm; } velo_t;

static void velo_step(velo_t *v, float target, bool held, float dt_ms) {
    // t_ms is the ramp clock: it advances while held (full travel in RAMP_UP) and
    // rewinds when released (full travel in RAMP_DOWN). Speed is a smoothstep
    // S-curve of the clock — non-linear, smooth at both ends, and a release/re-press
    // resumes seamlessly from the current point.
    if (held) v->t_ms += dt_ms;
    else      v->t_ms -= dt_ms * (float)EC_RAMP_UP_MS / (float)EC_RAMP_DOWN_MS;
    if (v->t_ms < 0.0f) v->t_ms = 0.0f;
    if (v->t_ms > (float)EC_RAMP_UP_MS) v->t_ms = (float)EC_RAMP_UP_MS;
    float f = v->t_ms / (float)EC_RAMP_UP_MS;
    v->rpm = target * f * (2.0f - f);                  // ease-out: quick start, smooth top
}


// ── Master task ───────────────────────────────────────────────────────────────
static void ethercat_task(void *arg) {
    (void)arg;
    NET_DBG("[EC] === EtherCAT master on core %u ===\n", (unsigned)get_core_num());

    if (!rmii_mac_init(s_mac)) {
        LOGE("[EC] LAN8720 MAC init failed (no PHY) — halting\n");
        for (;;) vTaskDelay(pdMS_TO_TICKS(2000));
    }

    uint8_t  out[PD_OUT_LEN];
    uint8_t  in[PD_IN_LEN] = {0};      // zero so the first cycle reads "not enabled"
    velo_t   velo[EC_DRIVE_COUNT] = {0};
    double   tpos[EC_DRIVE_COUNT] = {0};      // integrated target position (counts)
    bool     primed[EC_DRIVE_COUNT] = {0};    // tpos has a valid actual-position baseline
    int32_t  la_pos[EC_DRIVE_COUNT] = {0};    // last position sample (for measured rpm)
    uint32_t la_ms[EC_DRIVE_COUNT]  = {0};
    float    act_rpm[EC_DRIVE_COUNT] = {0};   // measured speed from position delta
    bool     up = false;
    uint32_t last_scan = 0, last_stats = 0, last_op = 0, last_slog = 0, last_cyc = 0;

    for (;;) {
        uint32_t now = now_ms();

        bool link = ec_link_up();
        SHARED_LOCK(); g_shared.link_up = link; SHARED_UNLOCK();
        if (!link) {
            if (up) LOGE("[EC] *** link lost ***\n");
            up = false;
            for (int d = 0; d < EC_DRIVE_COUNT; d++) set_phase(d, DS_SCAN, "No link");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (!up) {
            if (now - last_scan >= 1000) {
                last_scan = now;
                if (scan_and_bringup()) {
                    for (int d = 0; d < EC_DRIVE_COUNT; d++) {
                        velo[d].t_ms = velo[d].rpm = 0.0f;
                        tpos[d] = 0.0; primed[d] = false; la_ms[d] = 0; act_rpm[d] = 0.0f;
                    }
                    up = true;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Run requests come from the button poller on core 0 (via shared state),
        // or are forced on in autonomous bench-test mode.
        bool run_req[EC_DRIVE_COUNT];
        SHARED_LOCK();
        for (int d = 0; d < EC_DRIVE_COUNT; d++) run_req[d] = g_shared.drive[d].run_req;
        SHARED_UNLOCK();

        // Real elapsed time since the last cyclic pass — used so the ramp and the
        // position integration are wall-clock accurate regardless of the actual
        // loop rate (SPI overhead makes it < the nominal 1/EC_CYCLE_MS).
        float dt_ms = (last_cyc && now >= last_cyc) ? (float)(now - last_cyc) : (float)EC_CYCLE_MS;
        if (dt_ms > 50.0f) dt_ms = 50.0f;     // clamp a stall so we don't lurch
        last_cyc = now;

        // Build the output process image (per slave position → per drive index).
        // The controlword follows the CiA-402 enable state machine, gated by the
        // statusword from the previous cycle's input image.
        memset(out, 0, sizeof(out));
        for (int pos = 0; pos < s_npos; pos++) {
            if (!s_slave[pos].present) continue;
            int d = s_slave[pos].drive_idx;
            uint16_t sw = rd16(in + pos * PD_IN_PER_DRV);     // statusword (0x6041)

            uint16_t cw; bool enabled = false;
            if      (sw & 0x0008)            cw = 0x0080;     // FAULT → reset
            else if ((sw & 0x006F) == 0x0040) cw = 0x0006;    // switch-on-disabled → shutdown
            else if ((sw & 0x006F) == 0x0021) cw = 0x0007;    // ready → switch on
            else if ((sw & 0x006F) == 0x0023) cw = 0x000F;    // switched on → enable op
            else if ((sw & 0x006F) == 0x0027) { cw = 0x000F; enabled = true; }  // operation enabled
            else                              cw = 0x0006;

            bool want = (EC_AUTORUN ? true : run_req[d]) && s_slave[pos].op;
            float target = EC_AUTORUN ? (float)EC_AUTORUN_RPM : (float)EC_TARGET_RPM;
            velo_step(&velo[d], target, want && enabled, dt_ms);  // master-side accel/decel ramp

            // Position mode (CSP): once enabled, integrate the velocity ramp into
            // the target position (0x607A) using the REAL elapsed time. Baseline is
            // taken from the fresh actual position in the publish stage (and held
            // there until enabled), so enabling never jumps / faults.
            if (enabled)
                tpos[d] += (double)velo[d].rpm * (double)EC_COUNTS_PER_REV
                           * (double)dt_ms / 60000.0;

            uint8_t *o = out + pos * PD_OUT_PER_DRV;
            wr16(o, cw);
            wr32(o + 2, (uint32_t)(int32_t)tpos[d]);          // target position (0x607A)
        }

        uint16_t wkc = ec_exchange(out, in);
        if (wkc == 0) {
            LOGE("[EC] cyclic WKC=0 — slaves dropped, rescanning\n");
            up = false;
            for (int d = 0; d < EC_DRIVE_COUNT; d++) set_phase(d, DS_SCAN, "Lost - rescan");
            continue;
        }

        // Bring any not-yet-OP slave to OP now that valid process data is flowing.
        if (now - last_op >= 200) {
            last_op = now;
            for (int pos = 0; pos < s_npos; pos++) {
                if (!s_slave[pos].present || s_slave[pos].op) continue;
                uint8_t c[2]; wr16(c, AL_OP); ec_fpwr(s_slave[pos].station, REG_AL_CTRL, c, 2);
                uint8_t s[2] = {0,0};
                if (ec_fprd(s_slave[pos].station, REG_AL_STATUS, s, 2)) {
                    uint16_t als = rd16(s);
                    if ((als & 0x0F) == AL_OP) { s_slave[pos].op = true; LOGW("[EC] sta 0x%04X reached OP\n", s_slave[pos].station); }
                    else if (als & AL_ERR) {
                        uint8_t code[2] = {0,0}; ec_fprd(s_slave[pos].station, REG_AL_CODE, code, 2);
                        LOGE("[EC] sta 0x%04X OP refused AL=0x%02X code=0x%04X\n", s_slave[pos].station, als, rd16(code));
                        uint8_t ack[2]; wr16(ack, AL_OP | AL_ERR); ec_fpwr(s_slave[pos].station, REG_AL_CTRL, ack, 2);
                    }
                }
            }
        }

        for (int pos = 0; pos < s_npos; pos++) {
            if (!s_slave[pos].present) continue;
            int d = s_slave[pos].drive_idx;
            const uint8_t *ip = in + pos * PD_IN_PER_DRV;
            int32_t pos_in = rd32s(ip + 2);

            // Keep the target-position baseline pinned to the actual position until
            // the drive is enabled (and always on the first valid sample), so the
            // ramp integrates from where the motor really is — no jump on enable.
            bool en_now = (rd16(ip) & 0x006F) == 0x0027;
            if (!primed[d] || !en_now) { tpos[d] = (double)pos_in; primed[d] = true; }

            // Measured speed = position delta (0x6064) over a ~200 ms window → rpm,
            // then lightly smoothed (EMA) so the readout doesn't jitter.
            if (now - la_ms[d] >= 200) {
                if (la_ms[d] != 0) {
                    int32_t dp = pos_in - la_pos[d];
                    float dt = (float)(now - la_ms[d]);
                    float inst = (float)dp * 60000.0f / ((float)EC_COUNTS_PER_REV * dt);
                    act_rpm[d] = act_rpm[d] * 0.6f + inst * 0.4f;
                }
                la_pos[d] = pos_in; la_ms[d] = now;
            }
            bool running = velo[d].rpm > 0.0f;
            int32_t arpm = (int32_t)(act_rpm[d] >= 0 ? act_rpm[d] + 0.5f : act_rpm[d] - 0.5f);
            SHARED_LOCK();
            g_shared.drive[d].phase        = running ? DS_RUNNING : DS_CONNECTED;
            g_shared.drive[d].position     = pos_in;
            g_shared.drive[d].velocity_rpm = arpm;       // REAL speed from feedback
            SHARED_UNLOCK();
        }

        // Motion trace — ONE line per second per drive (not every cyclic send).
        if (now - last_slog >= 1000) {
            last_slog = now;
            for (int pos = 0; pos < s_npos; pos++) {
                if (!s_slave[pos].present) continue;
                int d = s_slave[pos].drive_idx;
                const uint8_t *ip = in + pos * PD_IN_PER_DRV;
                LOGW("[EC] sta 0x%04X %s SW=0x%04X pos=%ld vel=%drpm tgt=%ld wkc=%u\n",
                    s_slave[pos].station, s_slave[pos].op ? "OP" : "SAFEOP",
                    rd16(ip), (long)rd32s(ip + 2),
                    (int)(act_rpm[d] + (act_rpm[d] >= 0 ? 0.5f : -0.5f)),
                    (long)(int32_t)tpos[d], wkc);
            }
        }

        if (now - last_stats >= 1000) {
            last_stats = now;
            SHARED_LOCK();
            g_shared.heap_free = xPortGetFreeHeapSize();
            g_shared.uptime_s  = xTaskGetTickCount() / configTICK_RATE_HZ;
            SHARED_UNLOCK();
        }

        vTaskDelay(pdMS_TO_TICKS(EC_CYCLE_MS));
    }
}

void ethercat_start(UBaseType_t priority) {
    TaskHandle_t h = NULL;
    BaseType_t r = xTaskCreate(ethercat_task, "ecat", 4096, NULL, priority, &h);
    configASSERT(r == pdPASS);
    vTaskCoreAffinitySet(h, (UBaseType_t)(1u << 1));   // core 1 — MAC + drives
}
