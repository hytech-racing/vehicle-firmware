#ifndef ACU_INTERFACETASKS
#define ACU_INTERFACETASKS

#include "ACU_Constants.hpp"

/* External Includes */
#include "CANInterface.h"

#include <ht_task.hpp>

/* Local Interface Includes */
#include "ACUCANInterfaceImpl.hpp"
#include "ACUEthernetInterface.hpp"
#include "ADCInterface.hpp"
#include "BMSDriverGroup.hpp"
#include "BMSFaultDataManager.hpp"
#include "DataLoggingInterface.hpp"
#include "MAX114XInterface.hpp"
#include "SoHPersistenceInterface.hpp"
#include "SystemTimeInterface.h"
#include "VCRInterface.hpp"
#include "WatchdogInterface.hpp"

/* Local System Includes */
#include "ACUController.hpp"
#include "FaultLatchManager.hpp"
#include "WatchdogMetrics.hpp"

/* For Debugging */
#include "ACUStateMachine.hpp"

/* Scheduling */
#include <chrono>
#include <ht_task.hpp>

using chip_type = LTC6811_Type_e;
using BMSDriverInstance_t = BMSDriverInstance<ACUConstants::NUM_CHIPS, ACUConstants::NUM_CHIP_SELECTS, chip_type::LTC6811_1>;
using BMSFaultDataManagerInstance_t = BMSFaultDataManagerInstance<ACUConstants::NUM_CHIPS>;
// using MAX1148ADCInstance_t = MAX114XInterfaceInstance<ACUConstants::NUM_MAX1148_CHANNELS, ACUInterfaces::MAX114X_VERSION>;


/**
 * Init Functions - to be called in setup@
 */
void initialize_all_interfaces();

/**
 * This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
::HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sample_bms_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse write_cell_balancing_config(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse handle_send_ACU_core_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse handle_send_ACU_all_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_core_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_all_voltages_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_all_temps_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_ACU_ok_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueue_EM_measurement_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sample_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sample_adc(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse idle_sample_interfaces(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse init_soh_persistence(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse persist_soh_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse run_data_logging(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


template <typename bms_data> void print_bms_data(bms_data data);

#endif
