#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H

#include "SharedFirmwareTypes.h"
#include "VCR_Constants.hpp"
#include "VCR_Inverters.hpp"
#include "controls.hpp"

/* External Includes */
#include <ht_task.hpp>

/* Local System Includes */
#include "VehicleStateMachine.hpp"

/* Local Interface Includes */
#include "VCFInterface.hpp"
#include "ACUInterface.hpp"

#include "DrivetrainSystem.hpp"


/**
 * @brief Creates an instance of all systems.
 */
void initialize_all_systems();

#endif // __VCR_SYSTEMTASKS_H__
