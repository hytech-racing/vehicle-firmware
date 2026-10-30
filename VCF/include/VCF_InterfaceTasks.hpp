#ifndef VCF_INTERFACETASKS
#define VCF_INTERFACETASKS

#include "VCF_Constants.hpp"

/* External Includes */
#include "CANInterface.h"

#include <ht_task.hpp>

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "ADCInterface.hpp"
#include "BrakeRotorTempInterface.hpp"
#include "DashboardInterface.hpp"
#include "NeopixelInterface.hpp"
#include "OrbisInterface.hpp"
#include "SystemTimeInterface.h"
#include "VCFCANInterface.hpp"
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
 * The handle_send_VCF_ethernet_data task will send a protobuf message from VCF
 * to a destination port defined in EthernetAddressDefs. This function relies on
 * the VCF (sending) socket and vcf_data defined in VCFGlobals.h, and Ethernet
 * constants defined in EthernetAddressDefs.h.
 *
 */
HT_TASK::TaskResponse initSendAllETHDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse sendAllETHDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse run_dash_GPIOs_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info); // NOLINT (capitalization of GPIOs)

/**
 * @brief Tasks that pack outgoing CAN messages and enqueue them, plus the task that sends them.
 * All CAN TX goes through VCFCANInterface::enqueue(); tasks never touch the CAN buffers directly.
 * Enqueue tasks for system data are declared in VCF_SystemTasks.hpp under the same namespace.
 */
namespace CAN_tasks
{
    HT_TASK::TaskResponse enqueueDashboardMsgTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
    HT_TASK::TaskResponse enqueueFrontSuspensionMsgTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

    // this task attempts to send any data that is enqueued at 250hz. this will be the max rate that you can send over the CAN bus.
    // you dont have to enqeue at this rate, but this allows us to have 2 layers of rate limiting on CAN sending
    HT_TASK::TaskResponse clearCANBuffersTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalization of CAN)
} // namespace CAN_tasks


HT_TASK::TaskResponse debugPrints(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks
{
    // the others in the VCF Tasks can just stay there, they dont need forward declarations.
    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
} // namespace async_tasks

#endif
