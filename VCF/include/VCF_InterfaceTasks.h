#ifndef VCF_INTERFACETASKS
#define VCF_INTERFACETASKS

#include "VCF_Constants.h"

/* External Includes */
#include <ht_task.hpp>
#include "CANInterface.h"

/* Local Interface Includes */
#include "ACUInterface.h"
#include "ADCInterface.h"
#include "BrakeRotorTempInterface.h"
#include "DashboardInterface.h"
#include "OrbisInterface.h"
#include "SystemTimeInterface.h"
#include "VCFCANInterfaceImpl.h"
#include "VCFEthernetInterface.h"
#include "VCRInterface.h"
#include "WatchdogInterface.h"

/* Local System Includes */
#include "NeopixelController.h"


/**
 * Init Functions - to be called in setup@
 */
void initialize_all_interfaces();

/**
 * The read_adc0 task will command the ADCInterface to sample, convert, and store
 * data from all eight channels of adc0.
 */
::HT_TASK::TaskResponse run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * The read_adc0 task will command the ADCInterface to sample, convert, and store
 * data from all eight channels of adc1.
 */
::HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


::HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * The buzzer_control task will control the buzzer control pin. This function
 * relies on the buzzer_control pin definition in VCF_Constants.h;
 */
::HT_TASK::TaskResponse init_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
::HT_TASK::TaskResponse run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * The handle_send_VCF_ethernet_data task will send a protobuf message from VCF
 * to a destination port defined in EthernetAddressDefs. This function relies on
 * the VCF (sending) socket and vcf_data defined in VCFGlobals.h, and Ethernet
 * constants defined in EthernetAddressDefs.h.
 *
 */
HT_TASK::TaskResponse init_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

// this task attempts to send any data that is enqueued at 250hz. this will be the max rate that you can send over the CAN bus.
// you dont have to enqeue at this rate, but this allows us to have 2 layers of rate limiting on CAN sending
HT_TASK::TaskResponse handle_CAN_send(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo); // NOLINT (capitalization of CAN)

HT_TASK::TaskResponse run_dash_GPIOs_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info); // NOLINT (capitalization of GPIOs)
HT_TASK::TaskResponse send_dash_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

HT_TASK::TaskResponse enqueue_front_suspension_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks
{
    // the others in the VCF Tasks can just stay there, they dont need forward declarations.
    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
}

#endif
