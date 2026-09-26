#include "DrivetrainSystem.hpp"


DrivetrainStatus_s DrivetrainSystem::evaluate_drivetrain(DrivetrainCommand_s command, unsigned long current_millis)
{
    DrivetrainState_e current_dtsm_state = _evaluate_state_machine(current_millis);

    if (current_dtsm_state == DrivetrainState_e::READY)
    {
        if (command.control_mode == DrivetrainControlMode_e::TORQUE)
        {
            _setMotorsTorque(command, current_millis);
        }
        else if (command.control_mode == DrivetrainControlMode_e::SPEED)
        {
            _setMotorsSpeed(command, current_millis);
        }
    }

    DrivetrainStatus_s status = {};
    status.are_all_inverters_connected = _are_all_inverters_connected();
    status.are_all_inverters_enabled = _isDriveEnabled();
    status.is_control_mode_mismatch_detected = _isControlModeMismatched(current_millis);

    // This isn't really doing anything? How do I know that the command was actually okay rn. I should be checking something
    status.cmd_response = current_dtsm_state == DrivetrainState_e::NOT_CONNECTED
                                                ? DrivetrainCmdResponse_e::CANNOT_SEND_NOT_CONNECTED
                                                : DrivetrainCmdResponse_e::COMMAND_OK;

    status.current_dtsm_state = current_dtsm_state;

    status.inverter_statuses = {
        _inverter_interfaces_functs.FL.getInverterStatus(),
        _inverter_interfaces_functs.FR.getInverterStatus(),
        _inverter_interfaces_functs.RL.getInverterStatus(),
        _inverter_interfaces_functs.RR.getInverterStatus()
    };

    _status = status;
    return status;
}

DrivetrainState_e DrivetrainSystem::getCurrentState() const
{
    return _current_state;
}

DrivetrainStatus_s DrivetrainSystem::getStatus() const
{
    return _status;
}

const char* DrivetrainSystem::getStateName() const
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
        case DrivetrainState_e::READY:
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
    switch (getCurrentState())
    {
        case DrivetrainState_e::NOT_CONNECTED:
        {
            /**
             * @brief DEFAULT state, have not yet established CAN communication with all 4 inverters
             * @note We are assuming the DTI's continously send status messages on startup
             *
             * ERROR MODES :
             *  - No communication from ALL 4 inverters, but the ones we have communication with are faulted
             *  - Don't recieve CAN messages after X time
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _isAnyInverterFaulted();

            if (is_any_inverter_faulted)
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (are_all_inverters_connected)
            {
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }

            break;
        }
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        {
            /**
             * @brief CAN communication is established with all 4 inverters
             *        But, HV has not been established as present to the inverters
             * @note This state is the equivalent of TRACTIVE_SYSTEM_NOT_ACTIVE in the Vehicle State Machine
             *
             * ERROR MODES :
             *  - Lose connection with one or more inverters
             *  - Fault code from one or more inverters
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _isAnyInverterFaulted();
            bool is_hv_status_ok = _is_hv_status_ok();

            if (!are_all_inverters_connected)
            {
                // We will just enter NOT_CONNECTED; FAULTED state is only for fault codes
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (is_any_inverter_faulted)
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (is_hv_status_ok)
            {
                _setState(DrivetrainState_e::CONNECTED_HV_PRESENT, current_millis);
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
            bool is_any_inverter_faulted = _isAnyInverterFaulted();
            bool is_hv_status_ok = _is_hv_status_ok();
            bool is_drive_enabled = _isDriveEnabled();

            if (!are_all_inverters_connected)
            {
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (is_any_inverter_faulted)
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (!is_hv_status_ok)
            {
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
                // This is the equivalent of delatching before reaching RTD
            }
            else if (is_drive_enabled)
            {
                // This is the same as going into RTD, in the VSM, RTD state will set drive_enable
                _setState(DrivetrainState_e::READY, current_millis);
            }

            break;
        }
        case DrivetrainState_e::READY:
        {
            /**
             * @brief READY state: 1) Constant CAN communication with all 4 inverters
             *                      2) HV is present and OK
             *                      3) drive_enabled HIGH
             * @note This state is the equivalent of READY_TO_DRIVE in the Vehicle State Machine
             *
             * ERROR MODES
             *  - Lose connection with one or more inverters (fatal)
             *  - Fault code from one or more inverters
             *  - HV is no longer present (Delatch)
            */

            bool are_all_inverters_connected = _are_all_inverters_connected();
            bool is_any_inverter_faulted = _isAnyInverterFaulted();
            bool is_hv_status_ok = _is_hv_status_ok();
            bool is_drive_enabled = _isDriveEnabled();

            if (!are_all_inverters_connected)
            {
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (is_any_inverter_faulted)
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (!is_hv_status_ok)
            {
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }
            else if (!is_drive_enabled)
            {
                // if we lost drive_enable, lose RTD
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }

            break;
        }
        case DrivetrainState_e::FAULTED:
        {
            /**
             * @note DTI auto-clears faults internally once the underlying condition has cleared
             * ASSUMPTION: drive enabled is set high by the inverter when Fault Stop Timer is done
            */

            bool is_any_inverter_faulted = _isAnyInverterFaulted();
            bool is_drive_enabled = _isDriveEnabled();

            if (!is_any_inverter_faulted & is_drive_enabled)
            {
                _setState(DrivetrainState_e::WANTING_OK, current_millis);
            }

            break;
        }
        default:
        {
            break;
        }
    }

    return getCurrentState();
}

void DrivetrainSystem::_setState(DrivetrainState_e new_state, unsigned long current_millis)
{
    _handleExitLogic(_current_state, current_millis);
    _current_state = new_state;
    _handleEntryLogic(_current_state, current_millis);
    _last_state_changed_time = current_millis;
}

void DrivetrainSystem::_handleExitLogic(DrivetrainState_e prev_state, unsigned long current_millis)
{
    switch (prev_state)
    {
        case DrivetrainState_e::NOT_CONNECTED:
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        case DrivetrainState_e::WANTING_OK:
        case DrivetrainState_e::READY:
        case DrivetrainState_e::FAULTED:
        default:
            break;
    }
}

void DrivetrainSystem::_handleEntryLogic(DrivetrainState_e new_state, unsigned long current_millis)
{
    switch (new_state)
    {
        case DrivetrainState_e::NOT_CONNECTED:
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        case DrivetrainState_e::WANTING_OK:
        case DrivetrainState_e::FAULTED:
        {
            _setDriveEnable(false);
            break;
        }
        case DrivetrainState_e::READY:
        {
            _setDriveEnable(true);
            break;
        }
        default:
        {
            break;
        }
    }
}

bool DrivetrainSystem::_areAllInvertersConnected()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (!inverter_functs.getInverterStatus().is_inverter_connected)
        {
            return false;
        }
    }

    return true;
}

bool DrivetrainSystem::_isAnyInverterFaulted()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (inverter_functs.getInverterStatus().is_fault_code_present)
        {
            return true;
        }
    }

    return false;
}

bool DrivetrainSystem::_isDriveEnabled()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (!inverter_functs.getInverterStatus().is_drive_enabled)
        {
            return false;
        }
    }
    return true;
}

bool DrivetrainSystem::_isControlModeMismatched(unsigned long current_millis) const
{
    if ((current_millis - _last_control_mode_change_millis) < _control_mode_mismatch_threshold_ms)
    {
        return false;
    }

    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        if (!inverter_functs.isReportedModeMatchingDT(_last_commanded_control_mode))
        {
            return true;
        }
    }

    return false;
}

void DrivetrainSystem::_setDriveEnable(bool enable)
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        inverter_functs.requestEnable(enable);

        if (!enable)
        {
            inverter_functs.setMotorsIdle();
        }
    }
}

void DrivetrainSystem::_setMotorsTorque(DrivetrainCommand_s command, unsigned long current_millis)
{
    if (_last_commanded_control_mode != DrivetrainControlMode_e::TORQUE)
    {
        _last_commanded_control_mode = DrivetrainControlMode_e::TORQUE;
        _last_control_mode_change_millis = current_millis;
    }

    _inverter_interfaces_functs.FL.setMotorsTorque(command.desired_torques.FL);
    _inverter_interfaces_functs.FR.setMotorsTorque(command.desired_torques.FR);
    _inverter_interfaces_functs.RL.setMotorsTorque(command.desired_torques.RL);
    _inverter_interfaces_functs.RR.setMotorsTorque(command.desired_torques.RR);
}


void DrivetrainSystem::_setMotorsSpeed(DrivetrainCommand_s command, unsigned long current_millis)
{
    if (_last_commanded_control_mode != DrivetrainControlMode_e::SPEED)
    {
        _last_commanded_control_mode = DrivetrainControlMode_e::SPEED;
        _last_control_mode_change_millis = current_millis;
    }

    _inverter_interfaces_functs.FL.setMotorsSpeed(command.desired_speeds.FL);
    _inverter_interfaces_functs.FR.setMotorsSpeed(command.desired_speeds.FR);
    _inverter_interfaces_functs.RL.setMotorsSpeed(command.desired_speeds.RL);
    _inverter_interfaces_functs.RR.setMotorsSpeed(command.desired_speeds.RR);
}
