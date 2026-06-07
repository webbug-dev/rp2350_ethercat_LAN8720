#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "pico/types.h"

#define configUSE_PREEMPTION                    1
#define configUSE_TICKLESS_IDLE                 0
// Must match SYS_CLOCK_KHZ in board.h (set in main() before the scheduler).
// messani-style edge-synced RMII → 250 MHz system clock (clkdiv=1 PIO).
#define configCPU_CLOCK_HZ                      ((uint32_t) 250000000)
#define configTICK_RATE_HZ                      ((TickType_t) 1000)
#define configMAX_PRIORITIES                    32
#define configMINIMAL_STACK_SIZE                ((configSTACK_DEPTH_TYPE) 256)
#define configMAX_TASK_NAME_LEN                 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configSTACK_DEPTH_TYPE                  uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/* Memory */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ((size_t)(200 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP        0

/* Hooks */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* Run-time stats (using pico hardware timer) */
#define configGENERATE_RUN_TIME_STATS           1
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    1
extern void     stats_timer_init(void);
extern uint32_t stats_timer_get(void);
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() stats_timer_init()
#define portGET_RUN_TIME_COUNTER_VALUE()         stats_timer_get()

/* Co-routines */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Software timers */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            1024

/* Cortex-M33 interrupt priorities */
#define configKERNEL_INTERRUPT_PRIORITY                 255
#define configMAX_SYSCALL_INTERRUPT_PRIORITY            16

/* SMP — RP2350 dual-core */
#define configNUMBER_OF_CORES                   2
#define configTICK_CORE                         0
#define configRUN_MULTIPLE_PRIORITIES           1
#define configUSE_CORE_AFFINITY                 1
#define configUSE_PASSIVE_IDLE_HOOK             0
#define configUSE_TASK_PREEMPTION_DISABLE       0

/* RP2350 ARMv8-M / NTZ port specifics.  FPU is disabled: the SDK enables it
 * by default but the FreeRTOS context switch on this port does not save FPU
 * registers, so any task-side float would corrupt state. GCC emits softfloat
 * libcalls instead (LVGL + tzconv do a little float math). */
#define configENABLE_FPU                        0
#define configENABLE_MPU                        0
#define configENABLE_TRUSTZONE                  0
#define configRUN_FREERTOS_SECURE_ONLY          1

/* Pico SDK interop */
#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1

#include <assert.h>
#define configASSERT(x) assert(x)

/* Optional API */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xQueueGetMutexHolder            1

#endif /* FREERTOS_CONFIG_H */
