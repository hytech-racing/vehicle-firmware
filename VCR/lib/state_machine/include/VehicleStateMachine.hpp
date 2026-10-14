#ifndef VEHICLE_STATE_MACHINE_H
#define VEHICLE_STATE_MACHINE_H

/* ETL Library */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "Logger.h"


class VehicleStateMachine
{
public:

    VehicleStateMachine(
        etl::delegate<bool()> check_hv_over_threshold,
        etl::delegate<bool()> is_start_button_pressed,
        etl::delegate<bool()> is_brake_pressed,
        etl::delegate<bool()> check_drivetrain_error_ocurred,
        etl::delegate<bool()> check_drivetrain_ready,
        etl::delegate<void()> start_buzzer,
        etl::delegate<void()> recalibrate_pedals,
        etl::delegate<void(bool, bool)> command_drivetrain,
        etl::delegate<bool()> check_pedals_timeout,
        etl::delegate<void()> reset_pedals_timeout,
        etl::delegate<bool()> is_inverter_reset_button_pressed,
        etl::delegate<bool()> is_calibrate_pedals_button_pressed,
        etl::delegate<void()> reset_inverter_error,
        etl::delegate<void()> recalibrate_steering,
        etl::delegate<bool()> is_calibrate_steering_button_pressed,
        etl::delegate<bool()> check_steering_timeout,
        etl::delegate<void()> reset_steering_timeout
    ) :
        _check_hv_over_threshold(check_hv_over_threshold),
        _is_start_button_pressed(is_start_button_pressed),
        _is_brake_pressed(is_brake_pressed),
        _check_drivetrain_error_ocurred(check_drivetrain_error_ocurred),
        _check_drivetrain_ready(check_drivetrain_ready),
        _start_buzzer(start_buzzer),
        _send_recalibrate_pedals_message(recalibrate_pedals),
        _command_drivetrain(command_drivetrain),
        _check_pedals_timeout(check_pedals_timeout),
        _reset_pedals_timeout(reset_pedals_timeout),
        _is_inverter_reset_button_pressed(is_inverter_reset_button_pressed),
        _is_calibrate_pedals_button_pressed(is_calibrate_pedals_button_pressed),
        _reset_inverter_error(reset_inverter_error),
        _send_recalibrate_steering_message(recalibrate_steering),
        _is_calibrate_steering_button_pressed(is_calibrate_steering_button_pressed),
        _check_steering_timeout(check_steering_timeout),
        _reset_steering_timeout(reset_steering_timeout)
    {
        _current_state = VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE;
    }

    VehicleState_e tick_state_machine(unsigned long curr_time_millis);

    VehicleState_e get_state() const { return _current_state; }

private:

    VehicleState_e _current_state;

    /**
     * Timestamp when entering WANTING_RECALIBRATE_PEDALS to ensure we stay there
     * for 1000ms before actually sending the recalibration command.
     */
    uint32_t _last_entered_pedals_waiting_state_ms = 0;
    /**
     * Timestamp when entering WANTING_RECALIBRATE_STEERING to ensure 3000ms pass before calibrating.
     */
    uint32_t _last_entered_steering_waiting_state_ms = 0;

    void _set_state(VehicleState_e new_state, unsigned long current_time_millis);

    void _handle_entry_logic(VehicleState_e prev_state, unsigned long current_time_millis);

    void _handle_exit_logic(VehicleState_e new_state, unsigned long current_time_millis);

    /**
     * Lambdas necessary for state machine to work.
     */
    etl::delegate<bool()> _check_hv_over_threshold;
    etl::delegate<bool()> _is_start_button_pressed;
    etl::delegate<bool()> _is_brake_pressed;
    etl::delegate<bool()> _check_drivetrain_error_ocurred;
    etl::delegate<bool()> _check_drivetrain_ready;
    etl::delegate<void()> _start_buzzer;
    etl::delegate<void()> _send_recalibrate_pedals_message;
    etl::delegate<void(bool, bool)> _command_drivetrain; // Passes in true/false depending on whether we're in RTD or not.
    etl::delegate<bool()> _check_pedals_timeout;
    etl::delegate<void()> _reset_pedals_timeout;
    etl::delegate<bool()> _is_inverter_reset_button_pressed;
    etl::delegate<bool()> _is_calibrate_pedals_button_pressed;
    etl::delegate<void()> _reset_inverter_error;
    etl::delegate<void()> _send_recalibrate_steering_message;
    etl::delegate<bool()> _is_calibrate_steering_button_pressed;
    etl::delegate<bool()> _check_steering_timeout;
    etl::delegate<void()> _reset_steering_timeout;

};

using VehicleStateMachineInstance = etl::singleton<VehicleStateMachine>;

#endif // VEHICLE_STATE_MACHINE_H
