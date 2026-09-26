#ifndef VCR_INTERFACETASKS
#define VCR_INTERFACETASKS

#include "controls.hpp"
#include "VCR_Constants.hpp"
#include "VCR_Globals.hpp"
#include "VCR_Inverters.hpp"

/* External Includes */
#include <Logger.h>
#include <ht_task.hpp>
#include "CANInterface.h"

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "ADCInterface.hpp"
#include "DrivebrainInterface.hpp"
#include "FlowmeterInterface.hpp"
#include "MCP23017Interface.hpp"
#include "SystemTimeInterface.h"
#include "VCRCANInterfaceImpl.hpp"
#include "VCREthernetInterface.hpp"
#include "WatchdogInterface.hpp"

/* Local System Includes */
#include "VehicleStateMachine.hpp"


void initializeAllInterfaces();

/**
 * @brief Task will tick ADC 0: sample all eight channels, convert the outputs, and store them in structs
*/
::HT_TASK::TaskResponse readADC0Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task will tick ADC 1: sample all eight channels, convert the outputs, and store them in structs
 * @note These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING.
 *       You can edit this task manually to add sensors.
*/
::HT_TASK::TaskResponse readADC1Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task checks if we are actively recieving ACU data
 * @post Set SOFTWARE_OK low if heartbeat is not ok: not recieving updated ACU data
*/
::HT_TASK::TaskResponse updateACUHeartbeatTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task will "kick" watchdog
 * @note Method called fetches the state from WatchdogInterface and write it to the watchdog pin
*/
::HT_TASK::TaskResponse kickWatchdogTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Uses the I2C IOExpander to sense the shutdown line.
*/
::HT_TASK::TaskResponse readIOExpanderTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task reads the received pedals data and determines whether to turn on the brake light or not.
 */
::HT_TASK::TaskResponse updateBrakelightTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task determines the vehicle states and modes when motor cooling is activated
*/
::HT_TASK::TaskResponse enableMotorCoolingTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task determines the vehicle states and modes when inverter cooling is activated
*/
::HT_TASK::TaskResponse enableInverterCoolingTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/* -------------------- CAN TASKS -------------------- */
::HT_TASK::TaskResponse enqueueSuspensionCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
::HT_TASK::TaskResponse enqueueFlowmeterCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
::HT_TASK::TaskResponse enqueueCoolantTempCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task calls method to enqueue DRIVEBRAIN_LATENCY_STATUSES and DRIVEBRAIN_LATENCY_TIMES messages
 */
::HT_TASK::TaskResponse enqueueControlsCANDataTask(const unsigned long& sysMicro, const HT_TASK::TaskInfo& taskInfo);

/**
 *
*/
::HT_TASK::TaskResponse enqueueInverterCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task calls method to enqueue CAR_STATES message
 * @note This message is mainly used by Dashboard
*/
::HT_TASK::TaskResponse enqueueVehicleStateCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task calls method to clear the TX buffers for TELEM, INVERTER, and RAUX
*/
::HT_TASK::TaskResponse sendAllCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


/* -------------------- ETHERNET TASKS -------------------- */
::HT_TASK::TaskResponse sendAllETHDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized VCR)


::HT_TASK::TaskResponse debugPrintTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks
{
    ::HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
    VCRInterfaceData_s gather_latest_interface_data(CANInterfaces_s &can_interfaces);
}


#endif /* VCR_INTERFACETASKS */
