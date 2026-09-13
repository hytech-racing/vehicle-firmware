#ifndef VCR_INTERFACETASKS
#define VCR_INTERFACETASKS

#include "controls.h"
#include "VCR_Constants.h"
#include "VCR_Globals.h"
#include "VCR_Inverters.h"

/* External Includes */
#include <Logger.h>
#include <ht_task.hpp>
#include "CANInterface.h"

/* Local Interface Includes */
#include "ACUInterface.h"
#include "ADCInterface.h"
#include "DrivebrainInterface.h"
#include "FlowmeterInterface.h"
#include "MCP23017Interface.h"
#include "SystemTimeInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "VCREthernetInterface.h"
#include "WatchdogInterface.h"

/* Local System Includes */
#include "VehicleStateMachine.h"


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

/**
 * @brief Task calls another method to enqueue suspension CAN data to be sent on TELEM CAN
*/
::HT_TASK::TaskResponse enqueueSuspensionCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * @brief Task calls another method to enqueue flowmeter CAN data to be sent on TELEM CAN
*/
::HT_TASK::TaskResponse enqueueFlowmeterCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT

/**
 * Handles sending controls info for drivebrain (latencies and stuff)
 */
::HT_TASK::TaskResponse enqueue_controls_CAN_data(const unsigned long& sysMicro, const HT_TASK::TaskInfo& taskInfo);

/**
 * Handles sending of coolant temperature data
*/
::HT_TASK::TaskResponse enqueue_coolant_temp_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Enqueues all inverter CAN data. This will add all inverter data to the CAN queue, and then
 * the send_all_data task will empty the queue.
 */
::HT_TASK::TaskResponse enqueue_inverter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Enqueues all data needed for dashboard.
 */
::HT_TASK::TaskResponse enqueue_dashboard_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Sends all CAN data from the TX buffers of both telem and inverter CAN lines.
 */
::HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Task for sending all ethernet data
 */
::HT_TASK::TaskResponse handle_send_VCR_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized VCR)

::HT_TASK::TaskResponse debugPrintTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks
{
    ::HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
    VCRInterfaceData_s gather_latest_interface_data(CANInterfaces_s &can_interfaces);
}


#endif /* VCR_INTERFACETASKS */
