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
#include "DrivetrainSystem.hpp"
#include "VCFInterface.hpp"
#include "RSSInterface.hpp"


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
