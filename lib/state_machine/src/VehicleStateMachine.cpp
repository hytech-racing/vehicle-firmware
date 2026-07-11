#include "VehicleStateMachine.h"


VehicleState_e VehicleStateMachine::tick_state_machine(unsigned long current_millis)
{
    switch (_current_state)
    {
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        {
            if (_is_inverter_reset_button_pressed() && _check_drivetrain_error_ocurred())
            {
                _reset_inverter_error();
            }

            if (_check_hv_over_threshold())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            if (_is_calibrate_pedals_button_pressed())
            {
                _set_state(VehicleState_e::WANTING_RECALIBRATE_PEDALS, current_millis);
            }

            if (_is_calibrate_steering_button_pressed())
            {
                _set_state(VehicleState_e::WANTING_RECALIBRATE_STEERING, current_millis);
            }

            _command_drivetrain(false, false);

            break;
        }
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
        {
            if (_is_inverter_reset_button_pressed() && _check_drivetrain_error_ocurred())
            {
                _reset_inverter_error();
            }

            _command_drivetrain(false, false);

            if (!_check_hv_over_threshold())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            if (_is_start_button_pressed() && _is_brake_pressed())
            {
                _set_state(VehicleState_e::WANTING_READY_TO_DRIVE, current_millis);
                break;
            }

            break;
        }
        case VehicleState_e::WANTING_READY_TO_DRIVE:
        {
            _command_drivetrain(true, false);

            if (!_check_hv_over_threshold())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }
            if (_check_drivetrain_error_ocurred()) {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }
            if (_check_drivetrain_ready())
            {
                _set_state(VehicleState_e::READY_TO_DRIVE, current_millis);
                break;
            }

            break;
        }
        case VehicleState_e::READY_TO_DRIVE:
        {
            _command_drivetrain(true, true);

            if (!_check_hv_over_threshold())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
                break;
            }

            if (_check_drivetrain_error_ocurred())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            if (_check_pedals_timeout())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
                break;
            }

            if (_check_steering_timeout())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_ACTIVE, current_millis);
            }

            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_PEDALS:
        {
            _command_drivetrain(false, false);

            if (!_is_calibrate_pedals_button_pressed())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            if (_is_calibrate_pedals_button_pressed() && (current_millis - _last_entered_pedals_waiting_state_ms > 3000))
            {
                _set_state(VehicleState_e::RECALIBRATING_PEDALS, current_millis);
            }

            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_STEERING:
        {
            _command_drivetrain(false, false);

            if (!_is_calibrate_steering_button_pressed())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            if (_is_calibrate_steering_button_pressed() && (current_millis - _last_entered_steering_waiting_state_ms > 3000))
            {
                _set_state(VehicleState_e::RECALIBRATING_STEERING, current_millis);
            }

            break;
        }
        case VehicleState_e::RECALIBRATING_STEERING:
         {
            _command_drivetrain(false, false);

            if (!_is_calibrate_steering_button_pressed())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            if (_is_calibrate_steering_button_pressed())
            {
                _send_recalibrate_steering_message();
            }

            break;

        }
        case VehicleState_e::RECALIBRATING_PEDALS:
        {
            _command_drivetrain(false, false);

            if (!_is_calibrate_pedals_button_pressed())
            {
                _set_state(VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE, current_millis);
            }

            if (_is_calibrate_pedals_button_pressed())
            {
                _send_recalibrate_pedals_message();
            }

            break;
        }
        default:
        {
            break;
        }
    }
    return _current_state;
}

void VehicleStateMachine::_set_state(VehicleState_e new_state, unsigned long curr_millis)
{
    _handle_exit_logic(_current_state, curr_millis);
    _current_state = new_state;
    _handle_entry_logic(_current_state, curr_millis);
}

void VehicleStateMachine::_handle_exit_logic(VehicleState_e prev_state, unsigned long curr_millis)
{
    switch (prev_state)
    {
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        {
            break;
        }
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
        {
            break;
        }
        case VehicleState_e::WANTING_READY_TO_DRIVE:
        {
            break;
        }
        case VehicleState_e::READY_TO_DRIVE:
        {
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_PEDALS:
        {
            _last_entered_pedals_waiting_state_ms = 0;
            break;
        }
        case VehicleState_e::RECALIBRATING_PEDALS:
        {
            _last_entered_pedals_waiting_state_ms = 0;
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_STEERING:
        {
            _last_entered_steering_waiting_state_ms = 0;
            break;
        }
        case VehicleState_e::RECALIBRATING_STEERING:
        {
            _last_entered_steering_waiting_state_ms = 0;
            break;
        }
        default:
        {
            break;
        }
    }
}

void VehicleStateMachine::_handle_entry_logic(VehicleState_e new_state, unsigned long curr_millis)
{
    switch (new_state)
    {
        case VehicleState_e::TRACTIVE_SYSTEM_NOT_ACTIVE:
        {
            break;
        }
        case VehicleState_e::TRACTIVE_SYSTEM_ACTIVE:
        {
            break;
        }
        case VehicleState_e::WANTING_READY_TO_DRIVE:
        {
            break;
        }
        case VehicleState_e::READY_TO_DRIVE:
        {
            _start_buzzer();
            _reset_pedals_timeout();
            _reset_steering_timeout();
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_PEDALS:
        {
            _last_entered_pedals_waiting_state_ms = curr_millis;
            break;
        }
        case VehicleState_e::RECALIBRATING_PEDALS:
        {
            break;
        }
        case VehicleState_e::WANTING_RECALIBRATE_STEERING:
        {
            _last_entered_steering_waiting_state_ms = curr_millis;
            break;
        }
        case VehicleState_e::RECALIBRATING_STEERING:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

