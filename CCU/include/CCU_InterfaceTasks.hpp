#ifndef CCU_INTERFACETASKS
#define CCU_INTERFACETASKS

#include "CCU_Constants.hpp"

/* External Includes */
#include <ht_task.hpp>
#include "CANInterface.h"

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "ButtonInterface.hpp"
#include "CCUCANInterfaceImpl.hpp"
#include "CCUEthernetInterface.hpp"
#include "ChargerInterface.hpp"
#include "DisplayInterface.hpp"
#include "Level2Interface.hpp"
#include "RotaryEncoderInterface.hpp"
#include "SystemTimeInterface.h"
#include "WatchdogInterface.hpp"

/* Local System Includes */
#include "Level2System.hpp"
#include "MainChargeSystem.hpp"
#include "ChargerStateMachine.hpp"


/**
 * @brief Creates an instance of all interfaces. Init functions are called if necessary.
 */
void initialize_all_interfaces();

/**
 * @brief This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
 */
::HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse run_update_display_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse run_toggle_display_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse run_read_encoder_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse handle_enqueue_acu_can_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse handle_enqueue_charger_can_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse run_send_ethernet(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse run_receive_ethernet(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sample_can_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse handle_send_all_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse debug_prints(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


#endif