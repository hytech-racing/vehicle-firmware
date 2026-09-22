#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H

#include "SharedFirmwareTypes.h"
#include "VCR_Constants.h"
#include "VCR_Inverters.h"
#include "controls.h"

/* External Includes */
#include <ht_task.hpp>

/* Local System Includes */
#include "VehicleStateMachine.h"

/* Local Interface Includes */
#include "VCFInterface.h"

#include "DrivetrainSystem.h"


/**
 * @brief Creates an instance of all systems.
 */
void initialize_all_systems();

/* Delegate Functions */
extern ::etl::delegate<void(torque_nm)> setMotorTorque;
extern ::etl::delegate<void(speed_rpm)> setMotorSpeedl;
extern ::etl::delegate<void()> setMotorIdle;


extern ::etl::delegate<bool()> isVehicleLatched;
extern ::etl::delegate<bool()> isRTDPressed;
extern ::etl::delegate<void()> startBuzzer;
extern ::etl::delegate<bool()> isBrakePressed;
extern ::etl::delegate<bool()> isPedalsTimedOut;
extern ::etl::delegate<bool()> isSteeringTimedOut;
extern ::etl::delegate<void()> resetPedalsHeartbeat;
extern ::etl::delegate<void()> resetSteeringHeartbeat;
extern ::etl::delegate<bool()> isPedalsRecalibratePressed;
extern ::etl::delegate<bool()> isSteeringRecalibratePressed;
extern ::etl::delegate<void()> sendRecalibratePedalsMessage;
extern ::etl::delegate<void()> sendRecalibrateSteeringMessage;
extern ::etl::delegate<bool()> isDrivetrainFaulted;
extern ::etl::delegate<bool()> isDrivetrainNotConnected;
extern ::etl::delegate<void(bool, unsigned long)> handleDrivetrainCommand;


#endif // __VCR_SYSTEMTASKS_H__
