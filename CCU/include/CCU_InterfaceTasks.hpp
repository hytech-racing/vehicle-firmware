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
void initializeAllInterfaces();

/**
 * @brief This task will fetch the watchdog state from WatchdogSystem and write it to the watchdog pin.
*/
::HT_TASK::TaskResponse kickWatchdogTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse updateDisplayTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse toggleDisplayTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse readEncoderTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueueACUCANDataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse enqueueChargerCANdataTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sendETHTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse receiveETHTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse sampleCANTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse clearAllCANBuffersTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

::HT_TASK::TaskResponse debug_prints(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


#endif