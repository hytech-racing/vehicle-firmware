#ifndef VCF_SYSTEMTASKS
#define VCF_SYSTEMTASKS

#include "VCF_Constants.h"

/* External Includes */
#include <ht_task.hpp>

/* Local System Includes */
#include "BuzzerController.h"
#include "EEPROMUtilities.h"
#include "IOExpanderUtilities.h"
#include "NeopixelController.h"
#include "PedalsSystem.h"
#include "SteeringSystem.h"

/**
 * @brief Creates an instance of all systems.
 */
void initialize_all_systems();

::HT_TASK::TaskResponse enqueue_pedals_data(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info);

::HT_TASK::TaskResponse update_pedals_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_steering_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse update_steering_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse update_neopixels_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


#endif