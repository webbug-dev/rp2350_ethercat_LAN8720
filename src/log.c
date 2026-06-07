#include "log.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

// Each queued line is a fixed-size buffer (copied by value into the queue, so no
// pointer-lifetime games and no shared mutable state between producers).
#define LOG_LINE_MAX   160
#define LOG_QUEUE_LEN   96      // doubled — absorbs bigger bursts

typedef struct { char buf[LOG_LINE_MAX]; } log_line_t;

static QueueHandle_t     s_queue;                 // NULL until log_init()
static volatile int      s_level = LOG_LEVEL;     // runtime threshold
static volatile uint32_t s_dropped;               // lines lost to a full queue

void log_set_level(int level) { s_level = level; }
int  log_get_level(void)      { return s_level; }

// The sole owner of stdout: pulls finished lines off the queue and writes them
// (the only place a blocking USB-CDC write can happen).
static void logger_task(void *arg) {
    (void)arg;
    log_line_t line;
    for (;;) {
        if (xQueueReceive(s_queue, &line, portMAX_DELAY) == pdTRUE) {
            fputs(line.buf, stdout);
            // Surface any overflow so a silenced burst is never mistaken for calm.
            uint32_t d = __atomic_exchange_n(&s_dropped, 0, __ATOMIC_RELAXED);
            if (d)
                printf("[%8lu] [LOG] %lu records dropped\n",
                       (unsigned long)to_ms_since_boot(get_absolute_time()),
                       (unsigned long)d);
        }
    }
}

void log_init(unsigned int priority, unsigned int core_affinity_mask) {
    s_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(log_line_t));
    configASSERT(s_queue);
    TaskHandle_t h = NULL;
    BaseType_t r = xTaskCreate(logger_task, "log", 1024, NULL,
                               (UBaseType_t)priority, &h);
    configASSERT(r == pdPASS);
    vTaskCoreAffinitySet(h, (UBaseType_t)core_affinity_mask);
}

void log_vprintf(int level, const char *fmt, va_list ap) {
    if (level <= 0 || level > s_level) return;

    // Format the whole line — timestamp + body — in the caller's context so the
    // timestamp reflects when it was logged, not when it is later drained.
    log_line_t line;
    int n = snprintf(line.buf, sizeof line.buf, "[%8lu] ",
                     (unsigned long)to_ms_since_boot(get_absolute_time()));
    if (n < 0) n = 0;
    if (n > (int)sizeof(line.buf) - 1) n = (int)sizeof(line.buf) - 1;
    vsnprintf(line.buf + n, sizeof(line.buf) - (size_t)n, fmt, ap);

    if (s_queue) {
        // Non-blocking. On overflow, discard the OLDEST queued line and enqueue
        // this fresh one, so the log stays current instead of stalling on stale
        // data. Each discarded line is counted and surfaced by the drain task.
        if (xQueueSend(s_queue, &line, 0) != pdTRUE) {
            log_line_t discard;
            if (xQueueReceive(s_queue, &discard, 0) == pdTRUE)
                __atomic_add_fetch(&s_dropped, 1, __ATOMIC_RELAXED);
            if (xQueueSend(s_queue, &line, 0) != pdTRUE)   // still full (SMP race)
                __atomic_add_fetch(&s_dropped, 1, __ATOMIC_RELAXED);
        }
    } else {
        // Pre-init fallback (single-threaded early boot only).
        fputs(line.buf, stdout);
    }
}

void log_printf(int level, const char *fmt, ...) {
    if (level <= 0 || level > s_level) return;    // gate before formatting
    va_list ap;
    va_start(ap, fmt);
    log_vprintf(level, fmt, ap);
    va_end(ap);
}

void log_emergency(const char *fmt, ...) {
    // Fatal path: the drain task will never run again, so write straight to
    // stdout and block until it's out.
    printf("[%8lu] ", (unsigned long)to_ms_since_boot(get_absolute_time()));
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    fflush(stdout);
}
