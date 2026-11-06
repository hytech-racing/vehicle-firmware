#ifndef DASH_TASKS_HPP
#define DASH_TASKS_HPP

#include "Dash_Constants.hpp"
#include "Dash_Globals.hpp"

/* External Includes */
#include "SharedFirmwareTypes.h"

#include <ht_task.hpp>

/* Local Interface Includes */
#include "CANInterface.hpp"
#include "DashCANInterfaceImpl.hpp"
#include "DisplayInterface.hpp"
#include "HT_FDCAN.hpp"
#include "HT_SPI.hpp"
#include "NeopixelController.hpp"
#include "VCFInterface.hpp"
#include "VCRInterface.hpp"
#include "bitmaps.hpp"


void initalize_all_interfaces();

::HT_TASK::TaskResponse run_update_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
::HT_TASK::TaskResponse screen_refresh(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
::HT_TASK::TaskResponse can_read(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

#endif // DASH_TASKS_HPP