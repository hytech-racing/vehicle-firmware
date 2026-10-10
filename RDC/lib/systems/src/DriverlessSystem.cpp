#include "DriverlessSystem.h"
#include "RDC_Constants.h"
#include "SharedFirmwareTypes.h"

bool DriverlessSystem::startupCheckNoTS()
{
    // 1. check dv mission is selected
    // 2. EBS air pressure > thereshold
    // 3. Stop toggling watchdog -> ebs supervisor must fail
    EBSData_s ebs_data = _getEbsDataDelegate();

    if (_getDriverlessMissionDelegate() == DriverlessMission_e::OFF)
    {
        return false;
    }

    if (ebs_data.pressure_1 < RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI || ebs_data.pressure_2 < RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI)
    {
        return false;
    }

    if (_getWatchdogStatusDelegate())
    {
        _watchdog_been_ok = true;
        _startup_check_start_time_millis = _getCurrentMillisDelegate();
        _commandFDCDelegate(false, 0);
        return false;
    }

    if (_watchdog_been_ok && (_getCurrentMillisDelegate() - _startup_check_start_time_millis > RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS))
    {
        _commandFDCDelegate(true, 0);
        _no_ts_startup_ok = true;
        return true;
    }
    else
    {
        return false;
    }
}

bool DriverlessSystem::startupCheckTSActive()
{
    // Each solenoid gets its own window, starting when that phase starts.
    // The no-TS watchdog delay runs first, so the window cannot be anchored at boot.
    // Solenoid 1 is commanded alone until both axles build pressure. Solenoid 2 is checked on a later call.
    if (_ebs_line_1_ok)
    {
        return true;
    }

    const unsigned long now = _getCurrentMillisDelegate();
    const BrakeFluidPressureData_s brake_data = _getCurrentBrakeDataDelegate();
    const bool brakes_pressurized =
        brake_data.brake_fluid_pressure_data_front > RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI && brake_data.brake_fluid_pressure_data_rear > RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI;

    if (!_ebs_line_2_ok)
    {
        if (!_solenoid_1_timing)
        {
            _solenoid_1_start_time = now;
            _solenoid_1_timing = true;
        }

        if ((now - _solenoid_1_start_time) <= RDCConstants::SINGLE_SOLENOID_VALIDATION_DELAY_MS)
        {
            _commandFDCDelegate(true, 1);
            if (brakes_pressurized)
            {
                _ebs_line_2_ok = true;
                _solenoid_2_start_time = now;
            }
        }
        return false;
    }

    if ((now - _solenoid_2_start_time) <= RDCConstants::SINGLE_SOLENOID_VALIDATION_DELAY_MS)
    {
        _commandFDCDelegate(true, 2);
        if (brakes_pressurized)
        {
            _ebs_line_1_ok = true;
            return true;
        }
    }

    return false;
}

void DriverlessSystem::resetNoTSValues()
{
    _watchdog_been_ok = false;
    _startup_check_start_time_millis = 0;
    _no_ts_startup_ok = false;
}

void DriverlessSystem::resetTSActiveValues()
{
    _solenoid_1_start_time = 0;
    _solenoid_2_start_time = 0;
    _solenoid_1_timing = false;
    _ebs_line_1_ok = false;
    _ebs_line_2_ok = false;
}
