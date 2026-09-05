#include <DrivetrainSystem.h>


DrivetrainStatus_s DrivetrainSystem::evaluate_drivetrain(DrivetrainCommand_s command, unsigned long current_millis)
{
    DrivetrainState_e current_dtsm_state = _evaluate_state_machine(current_millis);
    DrivetrainStatus_s status;

    if (current_dtsm_state == DrivetrainState_e::DRIVE_ENABLED)
    {
        if (command.control_mode == DrivetrainControlMode_e::TORQUE)
        {
            _set_drivetrain_torque(command, current_millis);
        }
        else if (command.control_mode == DrivetrainControlMode_e::SPEED)
        {
            _set_drivetrain_speed(command, current_millis);
        }
    }

    status.are_all_inverters_connected = _are_all_inverters_connected();
    status.are_all_inverters_enabled = _is_drive_enabled();
    status.is_control_mode_mismatch_detected = is_control_mode_mismatched(current_millis);

    // This isn't really doing anything? How do I know that the command was actually okay rn. I should be checking something
    status.cmd_response = current_dtsm_state == DrivetrainState_e::NOT_CONNECTED
                        ? DrivetrainCmdResponse_e::CANNOT_SEND_NOT_CONNECTED
                        : DrivetrainCmdResponse_e::COMMAND_OK;

    status.current_dtsm_state = current_dtsm_state;

    status.inverter_statuses = {
        _inverter_interfaces_functs.FL.get_status(),
        _inverter_interfaces_functs.FR.get_status(),
        _inverter_interfaces_functs.RL.get_status(),
        _inverter_interfaces_functs.RR.get_status()
    };

    _status = status;
    return status;
}

DrivetrainState_e DrivetrainSystem::get_current_state() const
{
    return _current_state;
}

DrivetrainStatus_s DrivetrainSystem::get_status() const
{
    return _status;
}

const char* DrivetrainSystem::get_state_name() const
{
    switch (_current_state)
    {
        case DrivetrainState_e::NOT_CONNECTED:
        {
            return "INVERTERS ARE NOT CONNECTED";
        }
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        {
            return "INVERTERS ARE CONNECTED, HV IS NOT PRESENT/OK";
        }
        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        {
            return "INVERTERS ARE CONNECTED, HV IS PRESENT/OK, DRIVE IS NOT ENABLED";
        }
        case DrivetrainState_e::DRIVE_ENABLED:
        {
            return "INVERTERS ARE CONNECTED, HV IS PRESENT/OK, DRIVE IS ENABLED";
        }
        case DrivetrainState_e::FAULTED:
        {
            return "ONE OR MORE INVERTERS FAULTED";
        }
        default:
        {
            return "UNKNOWN";
        }
    }
}

DrivetrainState_e DrivetrainSystem::_evaluate_state_machine(unsigned long current_millis)
{

    switch (get_current_state())
    {
        case DrivetrainState_e::NOT_CONNECTED:
        {
            /**
             * @brief This is the DEFAULT state. In this state, we have not yet established CAN communication with all 4 inverters.
             * @note The transition away from this state is dependent on the DTI's continously sending status messages on startup.
             *       This is UNVERIFIED behavior.
             *
             * ERROR MODES :
             *  - Don't have communication from all 4 inverters, but the ones we have communication with are faulted
             *  - Don't recieve CAN messages after X time
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _is_any_inverter_faulted();

            if (is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (are_all_inverters_connected)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }

            break;
        }
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        {
            /**
             * @brief In this state, we have established CAN communication with all 4 inverters.
             *        But HV has not been established as present to the inverters
             * @note This state is the equivalent of TRACTIVE_SYSTEM_NOT_ACTIVE in the Vehicle State Machine
             *
             * ERROR MODES :
             *  - Lose connection with one or more inverters (fatal)
             *  - Fault code from one or more inverters
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _is_any_inverter_faulted();
            bool is_hv_status_ok = _is_hv_status_ok();

            if (!are_all_inverters_connected)
            {
                _set_state(DrivetrainState_e::NOT_CONNECTED, current_millis);
                // We will just enter NOT_CONNECTED. the FAULTED state is only for when the DTIs themselves
                // have a fault code present
            }
            else if (is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::FAULTED, current_millis);
            }

            else if (is_hv_status_ok)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_PRESENT, current_millis);
            }

            break;
        }

        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        {
            /**
             * @brief In this state, we have established CAN communication with all 4 inverters.
             *        And we have checked that our HV status is okay. But drive is NOT enabled.
             * @note This state is the equivalent of TRACTIVE_ACTIVE in the Vehicle State Machine
             *
             * ERROR MODES
             *  - Lose connection with one or more inverters (fatal)
             *  - Fault code from one or more inverters
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _is_any_inverter_faulted();
            bool is_hv_status_ok = _is_hv_status_ok();
            bool is_drive_enabled = _is_drive_enabled();

            if (!are_all_inverters_connected)
            {
                _set_state(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (!is_hv_status_ok)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
                // This is the equivalent of delatching before reaching RTD
            }
            else if (is_drive_enabled)
            {
                // This is the same as going into RTD, in the VSM, RTD state will set drive_enable
                _set_state(DrivetrainState_e::DRIVE_ENABLED, current_millis);
            }

            break;
        }
        case DrivetrainState_e::DRIVE_ENABLED:
        {
            /**
             * @brief In this state, we have established CAN communication with all 4 inverters.
             *        HV is present. Drive is enabled.
             * @note This state is the equivalent of READY_TO_DRIVE in the Vehicle State Machine
             *
             * ERROR MODES
             *  - Lose connection with one or more inverters (fatal)
             *  - Fault code from one or more inverters
             *  - HV is no longer present (Delatch)
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _is_any_inverter_faulted();
            bool is_hv_status_ok = _is_hv_status_ok();
            bool is_drive_enabled = _is_drive_enabled();

            if (!are_all_inverters_connected)
            {
                _set_state(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (!is_hv_status_ok)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }
            else if (!is_drive_enabled)
            {
                // if we lost drive_enable, lose RTD, then we have delatched
                _set_state(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }
            break;
        }
        case DrivetrainState_e::FAULTED:
        {
            /**
             * @note DTI auto-clears faults internally after its own Fault Stop Time countdown, once the
             *       underlying condition has cleared
             */
            bool is_any_inverter_faulted = _is_any_inverter_faulted();

            if (!is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }

            break;
        }
        default:
        {
            break;
        }
    }

    return get_current_state();
}

void DrivetrainSystem::_set_state(DrivetrainState_e new_state, unsigned long current_millis)
{
    _handle_exit_logic(_current_state, current_millis);
    _current_state = new_state;
    _handle_entry_logic(_current_state, current_millis);
    _last_state_changed_time = current_millis;
}

void DrivetrainSystem::_handle_exit_logic(DrivetrainState_e prev_state, unsigned long current_millis)
{
    switch (prev_state)
    {
        case DrivetrainState_e::NOT_CONNECTED:
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        case DrivetrainState_e::DRIVE_ENABLED:
        case DrivetrainState_e::FAULTED:
        default:
            break;
    }
}

void DrivetrainSystem::_handle_entry_logic(DrivetrainState_e new_state, unsigned long current_millis)
{
    switch (new_state)
    {
        case DrivetrainState_e::NOT_CONNECTED:
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        case DrivetrainState_e::FAULTED:
            _set_drive_enable_(false);
            break;

        case DrivetrainState_e::DRIVE_ENABLED:
            _set_drive_enable_(true);
            break;

        default:
            break;
    }
}

bool DrivetrainSystem::_are_all_inverters_connected()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (!inverter_functs.get_status().is_inverter_connected)
        {
            return false;
        }
    }

    return true;
}

bool DrivetrainSystem::_is_any_inverter_faulted()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (inverter_functs.get_status().is_fault_code_present)
        {
            return true;
        }
    }

    return false;
}

bool DrivetrainSystem::_is_drive_enabled()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (!inverter_functs.get_status().is_drive_enabled)
        {
            return false;
        }
    }
    return true;
}

bool DrivetrainSystem::is_control_mode_mismatched(unsigned long current_millis) const
{
    if ((current_millis - _last_control_mode_change_millis) < _control_mode_mismatch_threshold_ms)
    {
        return false;
    }

    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (!inverter_functs.is_reported_mode_matching_dt(_last_commanded_control_mode))
        {
            return true;
        }
    }

    return false;
}

void DrivetrainSystem::_set_drive_enable_(bool enable)
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        inverter_functs.request_enable(enable);

        if (!enable)
        {
            inverter_functs.set_motors_idle();
        }
    }
}

void DrivetrainSystem::_set_drivetrain_torque(DrivetrainCommand_s command, unsigned long current_millis)
{
    if (_last_commanded_control_mode != DrivetrainControlMode_e::TORQUE)
    {
        _last_commanded_control_mode = DrivetrainControlMode_e::TORQUE;
        _last_control_mode_change_millis = current_millis;
    }

    _inverter_interfaces_functs.FL.set_motors_torque(command.desired_torques.FL);
    _inverter_interfaces_functs.FR.set_motors_torque(command.desired_torques.FR);
    _inverter_interfaces_functs.RL.set_motors_torque(command.desired_torques.RL);
    _inverter_interfaces_functs.RR.set_motors_torque(command.desired_torques.RR);
}


void DrivetrainSystem::_set_drivetrain_speed(DrivetrainCommand_s command, unsigned long current_millis)
{
    if (_last_commanded_control_mode != DrivetrainControlMode_e::SPEED)
    {
        _last_commanded_control_mode = DrivetrainControlMode_e::SPEED;
        _last_control_mode_change_millis = current_millis;
    }

    _inverter_interfaces_functs.FL.set_motors_speed(command.desired_speeds.FL);
    _inverter_interfaces_functs.FR.set_motors_speed(command.desired_speeds.FR);
    _inverter_interfaces_functs.RL.set_motors_speed(command.desired_speeds.RL);
    _inverter_interfaces_functs.RR.set_motors_speed(command.desired_speeds.RR);
}
