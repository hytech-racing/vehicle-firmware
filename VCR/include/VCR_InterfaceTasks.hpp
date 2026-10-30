#ifndef VCR_INTERFACE_TASKS_HPP
#define VCR_INTERFACE_TASKS_HPP

#include "VCR_Constants.hpp"
#include "VCR_Globals.hpp"
#include "VCR_Inverters.hpp"
#include "controls.hpp"

/* External Includes */
#include "CANInterface.h"

#include <Logger.h>
#include <ht_task.hpp>

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "ADCInterface.hpp"
#include "ADS112Interface.hpp"
#include "DrivebrainInterface.hpp"
#include "FlowmeterInterface.hpp"
#include "MCP23017Interface.hpp"
#include "SystemTimeInterface.h"
#include "VCRCANInterfaceImpl.hpp"
#include "VCREthernetInterface.hpp"
#include "WatchdogInterface.hpp"

/* Local System Includes */
#include "VehicleStateMachine.hpp"


/**
 * Init Functions - to be called in setup@
 */
void initializeAllInterfaces();

/**
 * The read_adc0 task will command adc0 to sample all eight channels, convert the outputs, and
 * store them in structs defined in shared_firmware_types. This function relies on adc_0 being
 * defined in VCRGlobals.h.
 */
::HT_TASK::TaskResponse readADC0Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
 * manually to add sensors.
 *
 * The read_adc1 task will command adc1 to sample all eight channels, convert the outputs, and
 * store them in a struct defined in shared_firmware_types. This function relies on adc_1 being
 * defined in VCRGlobals.h.
 */
::HT_TASK::TaskResponse readADC1Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task will tick the maintenance panel ADC (ADS112U04) state machine
 * @note Non-blocking: each tick advances one step, so a full 4-channel sweep takes several ticks
 *       (~70 ms conversion wait per channel at 20 SPS)
 */
::HT_TASK::TaskResponse readMPBADCTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will tick the AMS system and will update the software shutdown if necessary.
 */
::HT_TASK::TaskResponse updateACUHeartbeatTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
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


::HT_TASK::TaskResponse enableMotorCoolingTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
::HT_TASK::TaskResponse enableInverterCoolingTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Handles sending of suspension CAN message data (load cell and shock pot data)
 */
::HT_TASK::TaskResponse enqueueSuspensionCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Handles sending flowmeter CAN message data
 */
::HT_TASK::TaskResponse enqueueFlowmeterCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT

/**
 * Handles sending controls info for drivebrain (latencies and stuff)
 */
::HT_TASK::TaskResponse enqueueControlsCANDataTask(const unsigned long& sysMicro, const HT_TASK::TaskInfo& taskInfo);

/**
 * Handles sending of coolant temperature data
 */
::HT_TASK::TaskResponse enqueueCoolantTempCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Enqueues all inverter CAN data. This will add all inverter data to the CAN queue, and then
 * the send_all_data task will empty the queue.
 */
::HT_TASK::TaskResponse enqueueInverterCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Enqueues all data needed for dashboard.
 */
::HT_TASK::TaskResponse enqueueDashboardCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Sends all CAN data from the TX buffers of both telem and inverter CAN lines.
 */
::HT_TASK::TaskResponse sendAllCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Task for sending all ethernet data
 */
::HT_TASK::TaskResponse sendAllETHDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized VCR)

/**
 * Debug prints
 */
::HT_TASK::TaskResponse debugPrintTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks
{
    ::HT_TASK::TaskResponse handleAsyncMain(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
    VCRInterfaceData_s gatherLatestInterfaceData(CANInterfaces_s& can_interfaces);
} // namespace async_tasks


#endif /* VCR_INTERFACETASKS */
