#include "CoreRDCCANInterface.h"
#include "ht_can.h"

void FDCInterface::receiveEbsPressure(const CAN_message_t &msg, unsigned long curr_millis)
{
    // TODO: Make the CAN message template add to ht can
    // basically we copy the pedals msg receive from VCR here

    EBS_DATA_t ebs_data;

    Unpack_EBS_DATA_ht_can(&ebs_data, &msg.buf[0], msg.len);

    _current_ebs_data.pressure_1 = ebs_data.ebs_air_pressure_1;
    _current_ebs_data.pressure_2 = ebs_data.ebs_air_pressure_2;
    _current_ebs_data.supervisor_ok = ebs_data.ebs_supervisor_ok;

    if (!_pressure_heartbeat_init_ok)
    {
        _pressure_heartbeat_init_ok = true;
    }
    _current_ebs_data.pressure_last_recv_millis = curr_millis;
}

void FDCInterface::receiveBrakeData(const CAN_message_t &msg, unsigned long curr_millis)
{
    BRAKE_PRESSURE_t brake_data;

    Unpack_BRAKE_PRESSURE_ht_can(&brake_data, &msg.buf[0], msg.len);

    _current_brake_data.brake_fluid_pressure_data_front = brake_data.brake_pressure_1;
    _current_brake_data.brake_fluid_pressure_data_rear = brake_data.brake_pressure_2;
}

void FDCInterface::receiveDVMission(const CAN_message_t &msg, unsigned long curr_millis)
{
    FDC_DRIVERLESS_MISSION_t dv_mission;

    Unpack_FDC_DRIVERLESS_MISSION_ht_can(&dv_mission, &msg.buf[0], msg.len);
    _current_dv_mission = static_cast<DriverlessMission_e>(dv_mission.driverless_mission);
}

void FDCInterface::setFDCControl(bool continue_toggle_ebs_supervisor, uint8_t activate_ebs_solenoid_id)
{
    _fdc_control.toggle_watchdog = continue_toggle_ebs_supervisor;
    _fdc_control.activate_ebs_solenoid_id = activate_ebs_solenoid_id;
}
