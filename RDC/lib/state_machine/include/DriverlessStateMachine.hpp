#ifndef DRIVERLESSSTATEMACHINE_H
#define DRIVERLESSSTATEMACHINE_H

#include <SharedFirmwareTypes.h>
#include <etl/delegate.h>
#include <etl/singleton.h>

class DriverlessStateMachine
{
    public:
    DriverlessStateMachine(
        etl::delegate<bool()> getDsmsOnDelegate,
        etl::delegate<VehicleState_e()> getVehicleStateDelegate,
        etl::delegate<bool()> runStartupNoTsLogicDelegate,
        etl::delegate<bool()> runStartupTsActiveLogicDelegate,
        etl::delegate<bool()> runEBSBrakePressureOkDelegate,
        etl::delegate<void()> runResetStartupNoTsValuesDelegate,
        etl::delegate<void()> runResetStartupTsActiveValuesDelegate)
        : _getDsmsOnDelegate(getDsmsOnDelegate),
          _getVehicleStateDelegate(getVehicleStateDelegate),
          _runStartupNoTsLogicDelegate(runStartupNoTsLogicDelegate),
          _runStartupTsActiveLogicDelegate(runStartupTsActiveLogicDelegate),
          _runEBSBrakePressureOkDelegate(runEBSBrakePressureOkDelegate),
          _runResetStartupNoTsValuesDelegate(runResetStartupNoTsValuesDelegate),
          _runResetStartupTsActiveValuesDelegate(runResetStartupTsActiveValuesDelegate)
    {
        _curr_driverless_system_state = DriverlessSystemState_e::OFF;
    }

    // bool getDriverlessSystemOk() {return _isDriverlessSystemOk;}

    DriverlessSystemState_e tickStateMachine(unsigned long curr_millis);

    DriverlessSystemState_e getState() { return _curr_driverless_system_state; }

    private:
    void _setState(DriverlessSystemState_e new_state, unsigned long curr_millis);
    //@brief _handleEntryLogic runs whenever you enter a new state to clear values to their defaults. This way if we need to run startup checks multiple times we dont need to cycle LV.
    void _handleEntryLogic(DriverlessSystemState_e new_state, unsigned long curr_millis);

    void _handleExitLogic(DriverlessSystemState_e prev_state, unsigned long curr_millis);

    DriverlessSystemState_e _curr_driverless_system_state;

    etl::delegate<bool()> _getDsmsOnDelegate;
    etl::delegate<VehicleState_e()> _getVehicleStateDelegate;

    etl::delegate<bool()> _runStartupNoTsLogicDelegate;
    etl::delegate<bool()> _runStartupTsActiveLogicDelegate;
    etl::delegate<bool()> _runEBSBrakePressureOkDelegate;

    etl::delegate<void()> _runResetStartupNoTsValuesDelegate;
    etl::delegate<void()> _runResetStartupTsActiveValuesDelegate;
};

using DriverlessStateMachineInstance = etl::singleton<DriverlessStateMachine>;
#endif // DRIVERLESSSYSTEM_H
