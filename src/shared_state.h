// shared_state.h — the single mutex-guarded blob shared between the EtherCAT
// master task (core 0) and the LVGL display task (core 1).
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "board.h"     // EC_DRIVE_COUNT

// Per-drive lifecycle, surfaced on its OLED window.
typedef enum {
    DS_SCAN = 0,     // master is scanning the segment for this drive
    DS_FOUND,        // drive answered; bringing it to OPERATIONAL
    DS_CONNECTED,    // OPERATIONAL — idle, reporting position
    DS_RUNNING,      // OPERATIONAL — running the velocity profile
    DS_FAULT,        // drive reported a fault / dropped off the bus
} drive_phase_t;

typedef struct {
    drive_phase_t phase;
    char     msg[28];          // free-text status line while scanning/faulted
    int32_t  position;         // actual position, encoder counts (CiA-402 0x6064)
    int32_t  velocity_rpm;     // current commanded velocity (rpm, signed)
    bool     run_req;          // run request — set on core 0 from the button (hold)
    char     model[16];        // decoded model name (from CoE product code)
} drive_state_t;

typedef struct {
    bool          link_up;                  // LAN8720 PHY link (cable present)
    drive_state_t drive[EC_DRIVE_COUNT];    // [EC_DRV_SERVO], [EC_DRV_STEP]
    size_t        heap_free;
    uint32_t      uptime_s;
} shared_state_t;

extern shared_state_t    g_shared;
extern SemaphoreHandle_t g_shared_mutex;

void shared_state_init(void);

// Convenience: take/give the shared mutex.
#define SHARED_LOCK()   xSemaphoreTake(g_shared_mutex, portMAX_DELAY)
#define SHARED_UNLOCK() xSemaphoreGive(g_shared_mutex)

// FreeRTOS run-time-stats timer hooks (referenced from FreeRTOSConfig.h).
void     stats_timer_init(void);
uint32_t stats_timer_get(void);
