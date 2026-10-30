#ifndef VCR_SYSTEM_TASKS_HPP
#define VCR_SYSTEM_TASKS_HPP

#include "VCR_Constants.hpp"
#include "VCR_Inverters.hpp"
#include "controls.hpp"

/* External Includes */
#include <ht_task.hpp>

/* Local System Includes */
#include "VehicleStateMachine.hpp"

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "DrivetrainSystem.hpp"
#include "RSSInterface.hpp"
#include "VCFInterface.hpp"


/**
 * @brief Creates an instance of all systems.
 */
void initializeAllSystems();

#endif // __VCR_SYSTEMTASKS_H__