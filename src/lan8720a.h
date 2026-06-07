// lan8720a.h — LAN8720A PHY MII management register map (subset used here).
#pragma once

#define LAN8720A_BASIC_CONTROL_REG               (0)
#define LAN8720A_BASIC_CONTROL_REG_DUPLEX_MODE   (1 <<  8)
#define LAN8720A_BASIC_CONTROL_REG_REST_AUTO_NEG (1 <<  9)
#define LAN8720A_BASIC_CONTROL_REG_AUTO_NEG_EN   (1 << 12)
#define LAN8720A_BASIC_CONTROL_REG_RESET         (1 << 15)

#define LAN8720A_BASIC_STATUS_REG                (1)
#define LAN8720A_BASIC_STATUS_REG_LINK_STATUS        (1 << 2)
#define LAN8720A_BASIC_STATUS_REG_AUTO_NEGO_COMPLETE (1 << 5)

#define LAN8720A_AUTO_NEGO_REG                   (4)
#define LAN8720A_AUTO_NEGO_REG_IEEE802_3         (0b00001 << 0)
#define LAN8720A_AUTO_NEGO_REG_10_ABI            (1 << 5)
#define LAN8720A_AUTO_NEGO_REG_10_FD_ABI         (1 << 6)
#define LAN8720A_AUTO_NEGO_REG_100_ABI           (1 << 7)
#define LAN8720A_AUTO_NEGO_REG_100_FD_ABI        (1 << 8)

// LAN8720 vendor-specific Special Control/Status register: speed/duplex indication.
#define LAN8720A_SPECIAL_CONTROL_STATUS_REG      (31)
#define LAN8720A_SCSR_SPEED_MASK                 (0b111 << 2)
#define LAN8720A_SCSR_SPEED_100_FD               (0b110 << 2)
#define LAN8720A_SCSR_SPEED_100_HD               (0b010 << 2)
