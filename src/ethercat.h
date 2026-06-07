#pragma once
#include "FreeRTOS.h"
#include "task.h"

// Start the EtherCAT master task. It owns the LAN8720 RMII MAC, scans for the
// two drives (Servo + Step), configures CiA-402 Cyclic-Sync-Position
// PDOs, and runs the cyclic process-data loop that streams the button-driven
// motion profile. Runs on core 1. Publishes everything it learns into g_shared.
void ethercat_start(UBaseType_t priority);
