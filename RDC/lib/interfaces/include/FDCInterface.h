#ifndef FDC_INTERFACE_H
#define FDC_INTERFACE_H

/* ETL Library */
#include "RDC_Constants.h"
#include <FlexCAN_T4.h>
#include <etl/singleton.h>

#include <SharedFirmwareTypes.h>
#include <ht_can.h>

struct BrakeData_s
{
    uint16_t brake_1_fluid_pressure;
    uint16_t brake_2_fluid_pressure;
};

struct FDCControl_s
{
    bool toggle_watchdog;
    uint8_t activate_ebs_solenoid_id;
};

class FDCInterface

{
    public:
    FDCInterface() = delete;

    FDCInterface(unsigned long init_millis, unsigned long max_heartbeat_timeout_ms)
        : _max_heartbeat_timeout_ms(max_heartbeat_timeout_ms)
    {
        _current_ebs_data.pressure_last_recv_millis = 0;
        _current_ebs_data.pressure_heartbeat_ok = false;
    };

    void receiveEbsPressure(const CAN_message_t &msg, unsigned long curr_millis);
    void receiveBrakeData(const CAN_message_t &msg, unsigned long curr_millis);
    void receiveDVMission(const CAN_message_t &msg, unsigned long curr_millis);

    // @brief the output of RDC with both driverless state and whether or not FDC should activate ebs solenoids manually.
    void setFDCControl(bool continue_toggle_ebs_supervisor, uint8_t activate_ebs_solenoid_id);

    void resetEbsHeartbeat() { _current_ebs_data.pressure_heartbeat_ok = true; }

    bool getEbsPressureOk() const { return _current_ebs_data.pressure_1 > RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI && _current_ebs_data.pressure_2 > RDCConstants::EBS_AIR_PRESSURE_THRESHOLD_PSI; }

    FDCControl_s getFDCControl() const { return _fdc_control; }
    EBSData_s getEbsData() const { return _current_ebs_data; }
    BrakeFluidPressureData_s getBrakeData() const { return _current_brake_data; }

    DriverlessMission_e getDriverlessMision() const { return _current_dv_mission; }
    bool getEbsSupervisorStatus() const { return _current_ebs_data.supervisor_ok; }

    private:
    EBSData_s _current_ebs_data;
    BrakeFluidPressureData_s _current_brake_data;
    DriverlessMission_e _current_dv_mission;
    FDCControl_s _fdc_control;

    unsigned long _max_heartbeat_timeout_ms;
    bool _pressure_heartbeat_init_ok = false;
};

using FDCInterfaceInstance = etl::singleton<FDCInterface>;

#endif
