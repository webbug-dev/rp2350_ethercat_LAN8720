#pragma once
#include "FreeRTOS.h"
#include "task.h"

// Start the LVGL display task (SH1107 OLED). Pinned to core 1 (with the logger).
void display_task_start(UBaseType_t priority);
