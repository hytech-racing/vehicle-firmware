#ifndef DRIVERLESSSYSTEM_H
#define DRIVERLESSSYSTEM_H

#include <SharedFirmwareTypes.h>
#include <etl/delegate.h>
#include <etl/singleton.h>

#include "RDC_Constants.hpp"

class DriverlessSystem
{
    public:
    DriverlessSystem(
        etl::delegate<void(bool, uint8_t)> commandFDCDelegate,
        etl::delegate<DriverlessMission_e()> getDriverlessMissionDelegate,
        etl::delegate<EBSData_s()> getEbsDataDelegate,
        etl::delegate<bool()> getWatchdogStatusDelegate,
        etl::delegate<unsigned long()> getCurrentMillisDelegate,
        etl::delegate<BrakeFluidPressureData_s()> getCurrentBrakeDataDelegate)
        : _commandFDCDelegate(commandFDCDelegate),
          _getDriverlessMissionDelegate(getDriverlessMissionDelegate),
          _getEbsDataDelegate(getEbsDataDelegate),
          _getWatchdogStatusDelegate(getWatchdogStatusDelegate),
          _getCurrentMillisDelegate(getCurrentMillisDelegate),
          _getCurrentBrakeDataDelegate(getCurrentBrakeDataDelegate)
    {
    }

    bool startupCheckNoTS();
    bool startupCheckTSActive();

    void resetNoTSValues();
    void resetTSActiveValues();

    private:
    etl::delegate<void(bool, uint8_t)> _commandFDCDelegate; // continue toggle watchdog, solenoid control
    etl::delegate<DriverlessMission_e()> _getDriverlessMissionDelegate;
    etl::delegate<EBSData_s()> _getEbsDataDelegate;
    etl::delegate<bool()> _getWatchdogStatusDelegate;
    etl::delegate<unsigned long()> _getCurrentMillisDelegate;
    etl::delegate<BrakeFluidPressureData_s()> _getCurrentBrakeDataDelegate;

    bool _watchdog_been_ok = false;
    unsigned long _startup_check_start_time_millis = 0;

    bool _no_ts_startup_ok = false;

    unsigned long _solenoid_1_start_time = 0;
    unsigned long _solenoid_2_start_time = 0;
    bool _solenoid_1_timing = false;
    // Solenoid 1 validates line 2, then solenoid 2 validates line 1. Flags persist across ticks.
    bool _ebs_line_1_ok = false;
    bool _ebs_line_2_ok = false;
};

using DriverlessSystemInstance = etl::singleton<DriverlessSystem>;

#endif // DRIVERLESSSYSTEM_H
