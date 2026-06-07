// rmii_mac.h — raw Layer-2 Ethernet MAC for a LAN8720 RMII module on RP2350
// PIO+DMA. TX/RX are edge-synchronised to the module's REF_CLK (GP22), derived
// from messani/pico-lan8720. Send/receive whole Ethernet frames (no FCS) — what
// the EtherCAT master needs. Call from a single task (the EtherCAT master task).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Initialise PIO/DMA/MDIO and bring the PHY up (auto-negotiate 100BASE-TX).
// `mac` is informational — a raw MAC has no address filter. Returns true if a
// PHY answered on the MDIO bus.
bool rmii_mac_init(const uint8_t mac[6]);

// Queue one Ethernet frame (dst+src+ethertype+payload, WITHOUT FCS). The
// preamble/SFD are prepended, the frame padded to 60 bytes and the FCS appended.
// Blocks only until the previous frame's DMA finishes. Returns bytes accepted.
int rmii_mac_send(const uint8_t *frame, size_t len);

// Fetch the next received frame into `buf` (up to `cap`). The FCS is verified
// and stripped; the returned length excludes it. Returns 0 if none pending or
// the FCS was bad. Non-blocking.
int rmii_mac_recv(uint8_t *buf, size_t cap);

// PHY link state / negotiated speed (Mbit/s, 0 if down), refreshed from MDIO.
bool rmii_mac_link_up(void);
int  rmii_mac_link_speed(void);

// Raw MDIO access + diagnostics.
uint16_t rmii_mac_mdio_read(unsigned phy, unsigned reg);
void     rmii_mac_mdio_write(unsigned phy, unsigned reg, unsigned val);
uint16_t rmii_mac_phy_bmsr(void);
int      rmii_mac_phy_addr(void);
void     rmii_mac_stats(uint32_t *tx, uint32_t *rx);

// Re-arm the RX path (PIO SM + DMA) from a clean state — call before re-scanning
// after a link glitch so a stuck mid-frame RX does not mangle replies.
void     rmii_mac_rx_restart(void);
