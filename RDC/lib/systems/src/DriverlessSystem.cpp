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

    if (_watchdog_been_ok && (_startup_check_start_time_millis - _getCurrentMillisDelegate() > RDCConstants::STARTUP_WATCHDOG_CHECK_DELAY_MS))
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
    // CAN message is activate the individual solenoids + some other stuff idk yet
    // Command stop actuating the first solenoid
    EBSData_s ebs_data = _getEbsDataDelegate();
    BrakeFluidPressureData_s brake_data = _getCurrentBrakeDataDelegate();

    bool ebs_line_1_ok = false;
    bool ebs_line_2_ok = false;

    if ((_getCurrentMillisDelegate() - _solenoid_1_start_time <= RDCConstants::SINGLE_SOLENOID_VALIDATION_DELAY_MS) && !ebs_line_2_ok)
    {
        _commandFDCDelegate(true, 1);
        if (brake_data.brake_fluid_pressure_data_front > RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI &&
            brake_data.brake_fluid_pressure_data_rear > RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI)
        {
            ebs_line_2_ok = true;
        }
    }
    if (ebs_line_2_ok)
    {
        if ((_getCurrentMillisDelegate() - _solenoid_2_start_time <= RDCConstants::SINGLE_SOLENOID_VALIDATION_DELAY_MS) && !ebs_line_1_ok)
        {
            _commandFDCDelegate(true, 2);
            if (brake_data.brake_fluid_pressure_data_front > RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI &&
                brake_data.brake_fluid_pressure_data_rear > RDCConstants::BRAKE_FLUID_PRESSURE_THRESHOLD_PSI)
            {
                ebs_line_1_ok = true;
                _solenoid_1_start_time = 0;
                _solenoid_2_start_time = 0;
                return true;
            }
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
}
