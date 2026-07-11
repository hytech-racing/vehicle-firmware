#ifndef DASH_TASKS_H
#define DASH_TASKS_H

#include "Dash_Constants.h"
#include "Dash_Globals.h"

/* External Includes */
#include "SharedFirmwareTypes.h"
#include <ht_task.hpp>

/* Local Interface Includes */
#include "bitmaps.h"
#include "CANInterface.h"
#include "DashCANInterfaceImpl.h"
#include "DisplayInterface.h"
#include "HT_FDCAN.h"
#include "HT_SPI.h"
#include "NeopixelController.h"
#include "VCFInterface.h"
#include "VCRInterface.h"


void initalize_all_interfaces();

::HT_TASK::TaskResponse run_update_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
::HT_TASK::TaskResponse screen_refresh(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
::HT_TASK::TaskResponse can_read(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

#endif /* DASH_TASKS_H */