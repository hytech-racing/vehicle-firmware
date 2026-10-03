#ifndef VCF_SYSTEMTASKS
#define VCF_SYSTEMTASKS

#include "VCF_Constants.hpp"

/* External Includes */
#include <ht_task.hpp>

/* Local System Includes */
#include "BuzzerController.hpp"
#include "EEPROMUtilities.hpp"
#include "MCP23017Interface.hpp"
#include "NeopixelController.hpp"
#include "PedalsSystem.hpp"
#include "SteeringSystem.hpp"

/**
 * @brief Creates an instance of all systems.
*/
void initializeAllSystems();

::HT_TASK::TaskResponse enqueuePedalsCANDataTask(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info);

::HT_TASK::TaskResponse updatePedalsCalibrationTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueueSteeringCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse updateSteeringCalibrationTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse updateNeopixelsTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


#endif