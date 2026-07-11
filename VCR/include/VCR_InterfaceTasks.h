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
#include "IOExpanderInterface.h"
#include "SystemTimeInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "VCREthernetInterface.h"
#include "WatchdogInterface.h"

/* Local System Includes */
#include "VehicleStateMachine.h"


/**
 * Init Functions - to be called in setup@
 */
void initialize_all_interfaces();

/**
 * The read_adc0 task will command adc0 to sample all eight channels, convert the outputs, and
 * store them in structs defined in shared_firmware_types. This function relies on adc_0 being
 * defined in VCRGlobals.h.
 */
::HT_TASK::TaskResponse run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
 * manually to add sensors.
 *
 * The read_adc1 task will command adc1 to sample all eight channels, convert the outputs, and
 * store them in a struct defined in shared_firmware_types. This function relies on adc_1 being
 * defined in VCRGlobals.h.
 */
::HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will tick the AMS system and will update the software shutdown if necessary.
 */
::HT_TASK::TaskResponse update_acu_heartbeat(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
::HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Uses the I2C IOExpander to sense the shutdown line.
 */
::HT_TASK::TaskResponse read_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * This task reads the received pedals data and determines whether to turn on the brake light or not.
 */
::HT_TASK::TaskResponse run_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


::HT_TASK::TaskResponse enable_motor_cooling(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enable_inverter_cooling(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * Handles sending of suspension CAN message data (load cell and shock pot data)
 */
::HT_TASK::TaskResponse enqueue_suspension_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalized CAN)

/**
 * Handles sending flowmeter CAN message data
 */
::HT_TASK::TaskResponse enqueue_flowmeter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT

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

::HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks
{
    ::HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
    VCRInterfaceData_s gather_latest_interface_data(CANInterfaces_s &can_interfaces);
}


#endif /* VCR_INTERFACETASKS */
