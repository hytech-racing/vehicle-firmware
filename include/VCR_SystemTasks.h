#ifndef VCR_SYSTEMTASKS_H
#define VCR_SYSTEMTASKS_H

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
extern ::etl::delegate<bool()> hv_over_threshold;
extern ::etl::delegate<bool()> start_button_pressed;
extern ::etl::delegate<bool()> brake_pressed;
extern ::etl::delegate<bool()> drivetrain_error_present;
extern ::etl::delegate<bool()> drivetrain_ready;
extern ::etl::delegate<void()> send_buzzer_start_message;
extern ::etl::delegate<void()> send_recalibrate_pedals_message;
extern ::etl::delegate<void()> handle_drivetrain_command;
extern ::etl::delegate<bool()> pedals_heartbeat_not_ok;
extern ::etl::delegate<void()> reset_pedals_heartbeat;
extern ::etl::delegate<bool()> drivetrain_reset_pressed;
extern ::etl::delegate<bool()> recalibrate_pedals_button_pressed;
extern ::etl::delegate<void()> reset_dt_error;
extern ::etl::delegate<void()> send_recalibrate_steering_message;
extern ::etl::delegate<bool()> recalibrate_steering_button_pressed;
extern ::etl::delegate<bool()> steering_heartbeat_not_ok;
extern ::etl::delegate<void()> reset_steering_heartbeat;


#endif // __VCR_SYSTEMTASKS_H__
