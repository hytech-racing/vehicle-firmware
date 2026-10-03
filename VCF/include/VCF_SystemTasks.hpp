#ifndef VCF_SYSTEMTASKS
#define VCF_SYSTEMTASKS

#include "VCF_Constants.hpp"

/* External Includes */
#include <ht_task.hpp>

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "DashboardInterface.hpp"
#include "VCFCANInterfaceImpl.hpp"
#include "VCRInterface.hpp"
#include "NeopixelInterface.hpp"

/* Local System Includes */
#include "BuzzerController.hpp"
#include "EEPROMUtilities.hpp"
#include "MCP23017Interface.hpp"
#include "BuzzerController.hpp"
#include "NeopixelController.hpp"
#include "PedalsSystem.hpp"
#include "SteeringSystem.hpp"

extern etl::delegate<void(uint16_t, uint32_t)> setPixelColor;
extern etl::delegate<void(uint8_t)> setBrightness;
extern etl::delegate<void()> show;
extern etl::delegate<bool()> arePedalsCalibrating;

/**
 * @brief Creates an instance of all systems.
*/
void initializeAllSystems();

::HT_TASK::TaskResponse enqueuePedalsCANDataTask(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info);

::HT_TASK::TaskResponse updatePedalsCalibrationTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueueSteeringCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse updateSteeringCalibrationTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse updateNeopixelsTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse init_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


#endif