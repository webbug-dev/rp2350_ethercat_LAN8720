#pragma once
// Shared gate for verbose traces from the LAN8720 MAC and the EtherCAT master.
// Controlled by NET_DEBUG_ENABLE in board.h. When it is 0, NET_DBG() compiles
// to nothing; failures and warnings (LOGE/LOGW) are emitted regardless.
#include "board.h"
#include "log.h"

#if NET_DEBUG_ENABLE
#define NET_DBG(...) LOG(__VA_ARGS__)
#else
#define NET_DBG(...) ((void)0)
#endif
