#include "DriverlessStateMachine.hpp"

DriverlessSystemState_e DriverlessStateMachine::tickStateMachine(unsigned long curr_millis)
{
    switch (_curr_driverless_system_state)
    {
    case DriverlessSystemState_e::OFF:
        if (_getDsmsOnDelegate())
        {
            _setState(DriverlessSystemState_e::STARTUP_NO_TS, curr_millis);
        }
        break;
    case DriverlessSystemState_e::STARTUP_NO_TS:
        if (_runStartupNoTsLogicDelegate() && _getVehicleStateDelegate() == VehicleState_e::TRACTIVE_SYSTEM_ACTIVE)
        {
            _setState(DriverlessSystemState_e::STARTUP_TS_ACTIVE, curr_millis);
        }
        break;
    case DriverlessSystemState_e::STARTUP_TS_ACTIVE:
        if (_getVehicleStateDelegate() == VehicleState_e::TRACTIVE_SYSTEM_ACTIVE && _runStartupTsActiveLogicDelegate())
        {
            _setState(DriverlessSystemState_e::READY, curr_millis);
        }
        break;
    case DriverlessSystemState_e::READY:
        if (_runEBSBrakePressureOkDelegate() && _getVehicleStateDelegate() == VehicleState_e::READY_TO_DRIVE)
        {
            _setState(DriverlessSystemState_e::DRIVING, curr_millis);
        }
        else if (!_runEBSBrakePressureOkDelegate())
        {
            _setState(DriverlessSystemState_e::EMERGENCY, curr_millis);
        }
        break;
    case DriverlessSystemState_e::DRIVING:
    default:
        break;
    }
    return _curr_driverless_system_state;
}

void DriverlessStateMachine::_setState(DriverlessSystemState_e new_state, unsigned long curr_millis)
{
    _handleExitLogic(_curr_driverless_system_state, curr_millis);
    _curr_driverless_system_state = new_state;
    _handleEntryLogic(_curr_driverless_system_state, curr_millis);
}

void DriverlessStateMachine::_handleEntryLogic(DriverlessSystemState_e new_state, unsigned long curr_millis)
{
    switch (new_state)
    {
    case DriverlessSystemState_e::OFF:
        break;
    case DriverlessSystemState_e::STARTUP_NO_TS:
        _runResetStartupNoTsValuesDelegate();
        break;
    case DriverlessSystemState_e::STARTUP_TS_ACTIVE:
        _runResetStartupTsActiveValuesDelegate();
        break;
    default:
        break;
    }
}

void DriverlessStateMachine::_handleExitLogic(DriverlessSystemState_e prev_state, unsigned long curr_millis)
{
    switch (prev_state)
    {
    case DriverlessSystemState_e::OFF:
        break;
    case DriverlessSystemState_e::STARTUP_NO_TS:
        break;
    default:
        break;
    }
}
