#include "DrivetrainSystem.hpp"


DrivetrainStatus_s DrivetrainSystem::evaluateDrivetrain(const DrivetrainCommand_s& command, unsigned long current_millis)
{
    // One snapshot of every inverter per tick, so all decisions below see the same data
    _refreshInverterStatuses(current_millis);

    DrivetrainState_e current_dtsm_state = _evaluateStateMachine(command, current_millis);

    bool is_command_valid = _isCommandValid(command);

    if (current_dtsm_state == DrivetrainState_e::READY)
    {
        if (!is_command_valid)
        {
            _setMotorsIdle();
        }
        else if (command.control_mode == DrivetrainControlMode_e::TORQUE)
        {
            _setMotorsTorque(command, current_millis);
        }
        else    // _isCommandValid() guarantees SPEED here
        {
            _setMotorsSpeed(command, current_millis);
        }
    }

    // Sent every tick in every state so drive-disable and zero-current frames keep reaching the inverters
    _sendInverterCommands();

    DrivetrainStatus_s status = {};
    status.are_all_inverters_connected = _areAllInvertersConnected();
    status.are_all_inverters_enabled = _isDriveEnabled();
    status.is_control_mode_mismatch_detected = (current_dtsm_state == DrivetrainState_e::READY) &&
                                               _isControlModeMismatched(current_millis);

    if (current_dtsm_state == DrivetrainState_e::NOT_CONNECTED)
    {
        status.cmd_response = DrivetrainCmdResponse_e::CANNOT_SEND_NOT_CONNECTED;
    }
    else if (!is_command_valid)
    {
        status.cmd_response = DrivetrainCmdResponse_e::COMMAND_INVALID;
    }
    else if ((current_dtsm_state != DrivetrainState_e::READY) && _isCommandNonZero(command))
    {
        // Asked for torque/speed while the drivetrain can't deliver it
        status.cmd_response = DrivetrainCmdResponse_e::COMMAND_INVALID;
    }
    else
    {
        status.cmd_response = DrivetrainCmdResponse_e::COMMAND_OK;
    }

    status.current_dtsm_state = current_dtsm_state;
    status.inverter_statuses = _inverter_statuses;

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
        case DrivetrainState_e::WANTING_OK:
        {
            return "DRIVE ENABLE REQUESTED, WAITING FOR ALL INVERTERS TO CONFIRM";
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

DrivetrainState_e DrivetrainSystem::_evaluateStateMachine(const DrivetrainCommand_s& command, unsigned long current_millis)
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
             *  - The inverters we do have communication with are faulted
            */

            if (_isAnyInverterFaulted())
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (_areAllInvertersConnected())
            {
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }

            break;
        }
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        {
            /**
             * @brief CAN communication is established with all 4 inverters, but HV is not present/OK
             * @note This state is the equivalent of TRACTIVE_SYSTEM_NOT_ACTIVE in the Vehicle State Machine
             *
             * ERROR MODES :
             *  - Lose connection with one or more inverters
             *  - Fault code from one or more inverters
            */

            if (!_areAllInvertersConnected())
            {
                // We will just enter NOT_CONNECTED; FAULTED state is only for fault codes
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (_isAnyInverterFaulted())
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (_isHVStatusOK())
            {
                _setState(DrivetrainState_e::CONNECTED_HV_PRESENT, current_millis);
            }

            break;
        }
        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        {
            /**
             * @brief CAN communication established with all 4 inverters and HV is OK, but drive is NOT enabled
             * @note This state is the equivalent of TRACTIVE_ACTIVE in the Vehicle State Machine
             *
             * ERROR MODES
             *  - Lose connection with one or more inverters (fatal)
             *  - Fault code from one or more inverters
             *  - HV is no longer present (Delatch before reaching RTD)
            */

            if (!_areAllInvertersConnected())
            {
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (_isAnyInverterFaulted())
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (!_isHVStatusOK())
            {
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }
            else if (command.is_drive_enable_requested)
            {
                // RTD requested: entry logic asks the inverters to enable, WANTING_OK waits for them to confirm
                _setState(DrivetrainState_e::WANTING_OK, current_millis);
            }

            break;
        }
        case DrivetrainState_e::WANTING_OK:
        {
            /**
             * @brief Drive enable has been sent to all inverters; waiting for all four to report it back
             *
             * ERROR MODES
             *  - Lose connection with one or more inverters (fatal)
             *  - Fault code from one or more inverters
             *  - HV is no longer present (Delatch)
             *  - Inverters never confirm within drive_enable_timeout_ms
            */

            if (!_areAllInvertersConnected())
            {
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (_isAnyInverterFaulted())
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (!_isHVStatusOK())
            {
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }
            else if (!command.is_drive_enable_requested)
            {
                _setState(DrivetrainState_e::CONNECTED_HV_PRESENT, current_millis);
            }
            else if (_isDriveEnabled())
            {
                _setState(DrivetrainState_e::READY, current_millis);
            }
            else if ((current_millis - _last_state_changed_time) > _drivetrain_params.drive_enable_timeout_ms)
            {
                /**
                 * @note Not a fault code, so not FAULTED. Back off to HV_PRESENT.
                 *       If the enable request is still true, the next tick re-enters WANTING_OK and retries.
                 *       The VSM is responsible for leaving RTD (dropping the request) when the drivetrain
                 *       isn't READY after its grace period, which ends the retries.
                */
                _setState(DrivetrainState_e::CONNECTED_HV_PRESENT, current_millis);
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
             *  - Drive enable request dropped, or an inverter drops enable on its own
            */

            if (!_areAllInvertersConnected())
            {
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
            }
            else if (_isAnyInverterFaulted())
            {
                _setState(DrivetrainState_e::FAULTED, current_millis);
            }
            else if (!_isHVStatusOK())
            {
                _setState(DrivetrainState_e::CONNECTED_HV_ABSENT, current_millis);
            }
            else if (!command.is_drive_enable_requested || !_isDriveEnabled())
            {
                // HV is still fine, so drop back one step rather than to HV_ABSENT
                _setState(DrivetrainState_e::CONNECTED_HV_PRESENT, current_millis);
            }

            break;
        }
        case DrivetrainState_e::FAULTED:
        {
            /**
             * @note DTI auto-clears faults internally once the underlying condition has cleared.
             *       Drive enable is held LOW for the whole time we are in FAULTED (entry logic), so recovery must not
             *       depend on the inverter reporting drive enabled.
             *       On recovery we re-climb the ladder from NOT_CONNECTED. The drivetrain will re-enable if the
             *       request is still true, so the VSM must leave RTD whenever the drivetrain leaves READY.
            */

            if (!_isAnyInverterFaulted())
            {
                _setState(DrivetrainState_e::NOT_CONNECTED, current_millis);
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
    (void)current_millis;

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
        case DrivetrainState_e::WANTING_OK:
        {
            // Pending setpoints are already zero from the previous (disabled) state's entry logic
            _setDriveEnable(true);
            break;
        }
        case DrivetrainState_e::READY:
        {
            /**
             * @note Drive enable is already requested from WANTING_OK. Restart the mismatch timer so the inverters
             *       get the full threshold to switch into the commanded mode.
            */
            _last_control_mode_change_millis = current_millis;
            break;
        }
        case DrivetrainState_e::NOT_CONNECTED:
        case DrivetrainState_e::CONNECTED_HV_ABSENT:
        case DrivetrainState_e::CONNECTED_HV_PRESENT:
        case DrivetrainState_e::FAULTED:
        default:
        {
            _setDriveEnable(false);
            break;
        }
    }
}

void DrivetrainSystem::_refreshInverterStatuses(unsigned long current_millis)
{
    _inverter_statuses.FL = _inverter_interfaces_functs.FL.getInverterStatus(current_millis);
    _inverter_statuses.FR = _inverter_interfaces_functs.FR.getInverterStatus(current_millis);
    _inverter_statuses.RL = _inverter_interfaces_functs.RL.getInverterStatus(current_millis);
    _inverter_statuses.RR = _inverter_interfaces_functs.RR.getInverterStatus(current_millis);
}

bool DrivetrainSystem::_areAllInvertersConnected() const
{
    for (const auto& inverter_status : _inverter_statuses.as_array())
    {
        if (!inverter_status.is_inverter_connected)
        {
            return false;
        }
    }

    return true;
}

bool DrivetrainSystem::_isAnyInverterFaulted() const
{
    for (const auto& inverter_status : _inverter_statuses.as_array())
    {
        if (inverter_status.is_inverter_connected && inverter_status.is_fault_code_present)
        {
            return true;
        }
    }

    return false;
}

bool DrivetrainSystem::_isDriveEnabled() const
{
    for (const auto& inverter_status : _inverter_statuses.as_array())
    {
        if (!inverter_status.is_drive_enabled)
        {
            return false;
        }
    }

    return true;
}

bool DrivetrainSystem::_isControlModeMismatched(unsigned long current_millis) const
{
    if ((current_millis - _last_control_mode_change_millis) < _drivetrain_params.control_mode_mismatch_threshold_ms)
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

bool DrivetrainSystem::_isCommandValid(const DrivetrainCommand_s& command) const
{
    switch (command.control_mode)
    {
        case DrivetrainControlMode_e::TORQUE:
        {
            for (const auto& torque : command.desired_torques.as_array())
            {
                if (!std::isfinite(torque))
                {
                    return false;
                }
            }
            return true;
        }
        case DrivetrainControlMode_e::SPEED:
        {
            for (const auto& speed : command.desired_speeds.as_array())
            {
                if (!std::isfinite(speed))
                {
                    return false;
                }
            }
            return true;
        }
        default:
        {
            return false;
        }
    }
}

bool DrivetrainSystem::_isCommandNonZero(const DrivetrainCommand_s& command) const
{
    switch (command.control_mode)
    {
        case DrivetrainControlMode_e::TORQUE:
        {
            for (const auto& torque : command.desired_torques.as_array())
            {
                if (torque != 0.0f)
                {
                    return true;
                }
            }
            return false;
        }
        case DrivetrainControlMode_e::SPEED:
        {
            for (const auto& speed : command.desired_speeds.as_array())
            {
                if (speed != 0.0f)
                {
                    return true;
                }
            }
            return false;
        }
        default:
        {
            return false;
        }
    }
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

void DrivetrainSystem::_setMotorsIdle()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        inverter_functs.setMotorsIdle();
    }
}

void DrivetrainSystem::_setMotorsTorque(const DrivetrainCommand_s& command, unsigned long current_millis)
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

void DrivetrainSystem::_setMotorsSpeed(const DrivetrainCommand_s& command, unsigned long current_millis)
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

void DrivetrainSystem::_sendInverterCommands()
{
    for (const auto& inverter_functs : _inverter_interfaces_functs.as_array())
    {
        inverter_functs.sendCommands(_last_commanded_control_mode);
    }
}