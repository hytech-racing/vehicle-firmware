#ifndef VCF_INTERFACETASKS
#define VCF_INTERFACETASKS

#include "VCF_Constants.hpp"

/* External Includes */
#include <ht_task.hpp>
#include "CANInterface.h"

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "ADCInterface.hpp"
#include "BrakeRotorTempInterface.hpp"
#include "DashboardInterface.hpp"
#include "OrbisInterface.hpp"
#include "SystemTimeInterface.h"
#include "VCFCANInterfaceImpl.hpp"
#include "VCFEthernetInterface.hpp"
#include "VCRInterface.hpp"
#include "WatchdogInterface.hpp"

/* Local System Includes */
#include "NeopixelController.hpp"


/**
 * @brief Create instances of all interfaces and call necessary init methods
 */
void initializeAllInterfaces();

/**
 * @brief Task will command the ADCInterface to sample, convert, and store data from all eight channels of ADC0
*/
::HT_TASK::TaskResponse readADC0Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * @brief Task will command the ADCInterface to sample, convert, and store data from all eight channels of ADC1
*/
::HT_TASK::TaskResponse readADC1Task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse kickWatchdogTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

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
HT_TASK::TaskResponse initSendAllETHDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse sendAllETHDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse run_dash_GPIOs_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info); // NOLINT (capitalization of GPIOs)
HT_TASK::TaskResponse enqueueDashboardCANDataTask(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);
HT_TASK::TaskResponse enqueueFrontSuspensionCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

// this task attempts to send any data that is enqueued at 250hz. this will be the max rate that you can send over the CAN bus.
// you dont have to enqeue at this rate, but this allows us to have 2 layers of rate limiting on CAN sending
HT_TASK::TaskResponse clearCANBuffersTask(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo); // NOLINT (capitalization of CAN)


HT_TASK::TaskResponse debugPrints(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks
{
    // the others in the VCF Tasks can just stay there, they dont need forward declarations.
    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
}

#endif
