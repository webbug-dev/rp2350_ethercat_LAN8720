// log.h — asynchronous, thread-safe, multi-core logging.
//
// Every LOG* call FORMATS its line (with a "[<ms>] " timestamp) in the caller's
// context and ENQUEUES it into a fixed ring queue — it never touches stdout and
// never blocks. A dedicated drain task (see log.c, pinned to core 1 next to the
// display) is the sole writer to USB CDC, emitting queued lines one at a time.
// Producers on either core can log concurrently; the FreeRTOS queue serialises
// them. If the queue is full the line is dropped (and counted) rather than
// blocking the producer — losing a debug line beats stalling the net loop.
//
// Levels: 0 = OFF (nothing at all), 1 = ERROR, 2 = WARN, 3 = INFO (verbose).
// The threshold defaults to LOG_LEVEL (compile-time, 3) and is changeable at
// runtime via log_set_level(). A line is dropped before formatting if its level
// exceeds the current threshold.
#pragma once
#include <stdarg.h>

#define LOG_LEVEL_OFF    0   // log nothing
#define LOG_LEVEL_ERROR  1   // failures, faults, lost connections
#define LOG_LEVEL_WARN   2   // notable state changes (link/lease/NTP/MQTT up-down)
#define LOG_LEVEL_INFO   3   // verbose: heartbeats, per-packet traces, boot detail

#ifndef LOG_LEVEL
#define LOG_LEVEL        3   // default runtime threshold
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Bring up the queue + drain task. Call once, early in boot, before the first
// log line. `priority` is the drain task's FreeRTOS priority; `core_affinity_mask`
// pins it (bit N = core N).
void log_init(unsigned int priority, unsigned int core_affinity_mask);

// Runtime threshold control (initialised to LOG_LEVEL).
void log_set_level(int level);
int  log_get_level(void);

// Format + enqueue one line (non-blocking, thread- & SMP-safe).
void log_printf(int level, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void log_vprintf(int level, const char *fmt, va_list ap);

// Synchronous, queue-bypassing line for fatal paths (panic hooks) where the
// drain task will never get to run. Blocks; use only when about to die.
void log_emergency(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

// Leveled macros. A bare LOG() is INFO (level 3) for source-compatibility.
#define LOGE(fmt, ...) log_printf(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) log_printf(LOG_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) log_printf(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define LOG(fmt, ...)  log_printf(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
