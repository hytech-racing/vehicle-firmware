#include <DrivetrainSystem.h>


DrivetrainStatus_s DrivetrainSystem::evaluate_drivetrain(DrivetrainCommand_s cmd, unsigned long current_millis)
{
    DrivetrainState_e current_state = _evaluate_state_machine(cmd, current_millis);
    DrivetrainStatus_s status;

    status.are_all_inverters_connected = _are_all_inverters_connected();
    status.are_all_inverters_enabled = _is_drive_enabled();

    status.cmd_response = current_state == DrivetrainState_e::NOT_CONNECTED
        ? DrivetrainCmdResponse_e::CANNOT_SEND_NOT_CONNECTED
        : DrivetrainCmdResponse_e::COMMAND_OK;

    status.current_dsm_state = current_state;

    status.inverter_statuses = {
        _inverter_interfaces.FL.get_status(),
        _inverter_interfaces.FR.get_status(),
        _inverter_interfaces.RL.get_status(),
        _inverter_interfaces.RR.get_status()
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
        case DrivetrainState_e::CONNECTED_HV_NOT_OK:
        {
            return "INVERTERS ARE CONNECTED, HV IS NOT OK";
        }
        case DrivetrainState_e::CONNECTED_HV_OK_NOT_ENABLED:
        {
            return "INVERTERS ARE CONNECTED, HV IS OK, DRIVE IS NOT ENABLED";
        }
        case DrivetrainState_e::DRIVE_ENABLED:
        {
            return "INVERTERS ARE CONNECTED, HV IS OK, DRIVE IS ENABLED";
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

DrivetrainState_e DrivetrainSystem::_evaluate_state_machine(DrivetrainCommand_s command, unsigned long current_millis)
{

    switch (get_current_state())
    {
        case DrivetrainState_e::NOT_CONNECTED:
        {
            /**
             * @brief In this state, we have not established CAN communication with all 4 inverters.
             *
             * ERROR MODES
             *  - Fault: It is possible that we only receive communication from a inverter, and we immediately get a fault
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _is_any_inverter_faulted();

            if (is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (are_all_inverters_connected)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_NOT_OK, current_millis);
            }

            break;
        }
        case DrivetrainState_e::CONNECTED_HV_NOT_OK:
        {
            /**
             * @brief In this state, we have established CAN communication with all 4 inverters.
             *        But we have not checked that our HV status is okay.
             *
             * ERROR MODES
             *  - Lose connection with one or more inverters (fatal)
             *  - Fault code from one or more inverters
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _is_any_inverter_faulted();
            bool is_hv_status_ok = _is_hv_status_ok();

            if (!are_all_inverters_connected)
            {
                _set_state(DrivetrainState_e::NOT_CONNECTED, current_millis);
                // Do we just want to go into NOT_CONNECTED
                // Should we go into some error state that is separate from fault. In this error state we would probably try to disconnect as much as possible?
            }
            else if (is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (is_hv_status_ok)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_OK_NOT_ENABLED, current_millis);
            }

            break;
        }

        case DrivetrainState_e::CONNECTED_HV_OK_NOT_ENABLED:
        {
            /**
             * @brief In this state, we have established CAN communication with all 4 inverters.
             *        And we have checked that our HV status is okay.
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
                _set_state(DrivetrainState_e::CONNECTED_HV_NOT_OK, current_millis);
            }
            else if (is_drive_enabled)
            {
                _set_state(DrivetrainState_e::DRIVE_ENABLED, current_millis);
            }

            break;
        }
        case DrivetrainState_e::DRIVE_ENABLED:
        {
            _set_drivetrain_command(command);

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
                _set_state(DrivetrainState_e::CONNECTED_HV_NOT_OK, current_millis);
            }
            else if (!is_drive_enabled)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_OK_NOT_ENABLED, current_millis);
            }
            break;
        }
        case DrivetrainState_e::FAULTED:
        {
            // DTI auto-clears faults internally after its own Fault Stop Time
            // countdown, once the underlying condition has cleared — nothing
            // for us to command here, we just poll and exit once clear.
            bool is_any_inverter_faulted = _is_any_inverter_faulted();

            if (!is_any_inverter_faulted)
            {
                _set_state(DrivetrainState_e::CONNECTED_HV_NOT_OK, current_millis);
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
        case DrivetrainState_e::CONNECTED_HV_NOT_OK:
        case DrivetrainState_e::CONNECTED_HV_OK_NOT_ENABLED:
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
        case DrivetrainState_e::CONNECTED_HV_NOT_OK:
        case DrivetrainState_e::CONNECTED_HV_OK_NOT_ENABLED:
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
    for (const auto& inverter : _inverter_interfaces.as_array())
    {
        if (!inverter.get_status().is_inverter_connected)
        {
            return false;
        }
    }

    return true;
}

bool DrivetrainSystem::_is_any_inverter_faulted()
{
    for (const auto& inverter : _inverter_interfaces.as_array())
    {
        if (inverter.get_status().is_fault_code_present)
        {
            return true;
        }
    }

    return false;
}

bool DrivetrainSystem::_is_drive_enabled()
{
    for (const auto& inverter : _inverter_interfaces.as_array())
    {
        if (!inverter.get_status().is_inverter_enabled)
        {
            return false;
        }
    }
    return true;
}

void DrivetrainSystem::_set_drive_enable_(bool enable)
{
    for (const auto& inverter : _inverter_interfaces.as_array())
    {
        inverter.request_enable(enable);

        if (!enable)
        {
            inverter.set_idle();
        }
    }
}

void DrivetrainSystem::_set_drivetrain_command(DrivetrainCommand_s cmd)
{
    _inverter_interfaces.FL.set_torque(cmd.torque_setpoints.FL);
    _inverter_interfaces.FR.set_torque(cmd.torque_setpoints.FR);
    _inverter_interfaces.RL.set_torque(cmd.torque_setpoints.RL);
    _inverter_interfaces.RR.set_torque(cmd.torque_setpoints.RR);
}