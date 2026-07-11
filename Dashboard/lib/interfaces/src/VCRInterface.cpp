#include "VCRInterface.h"


void VCRInterface::receive_inv_dynamics(const CAN_message_t &can_msg, unsigned long curr_millis)
{
    // Unpack the message
    INV1_DYNAMICS_t unpacked_msg;
    Unpack_INV1_DYNAMICS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _wheel_data.actual_power_watts = unpacked_msg.actual_power_w; // NOLINT (watts)
    _wheel_data.actual_torque_nm = HYTECH_actual_torque_nm_ro_fromS(unpacked_msg.actual_torque_nm_ro);
    _wheel_data.actual_speed_rpm = unpacked_msg.actual_speed_rpm;
    _wheel_data.new_data = true;
    _wheel_data.last_recv_millis = curr_millis;
}

void VCRInterface::receive_vehicle_state(const CAN_message_t &can_msg)
{
    CAR_STATES_t unpacked_msg;
    Unpack_CAR_STATES_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _vehicle_state_value = static_cast<VehicleState_e>(unpacked_msg.vehicle_state);
    _drivetrain_state_value = static_cast<DrivetrainState_e>(unpacked_msg.drivetrain_state);
    _is_db_in_ctrl = unpacked_msg.drivebrain_in_control;
}

void VCRInterface::receive_inverter_status_1(const CAN_message_t &can_msg)
{
    INV1_STATUS_t unpacked_msg;
    Unpack_INV1_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _inverter_status.error.FL = unpacked_msg.error;
    _inverter_status.dc_bus_voltage.FL = unpacked_msg.dc_bus_voltage;
}

void VCRInterface::receive_inverter_status_2(const CAN_message_t &can_msg)
{
    INV2_STATUS_t unpacked_msg;
    Unpack_INV2_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _inverter_status.error.FR = unpacked_msg.error;
    _inverter_status.dc_bus_voltage.FR = unpacked_msg.dc_bus_voltage;

}

void VCRInterface::receive_inverter_status_3(const CAN_message_t &can_msg)
{
    INV3_STATUS_t unpacked_msg;
    Unpack_INV3_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _inverter_status.error.RL = unpacked_msg.error;
    _inverter_status.dc_bus_voltage.RL = unpacked_msg.dc_bus_voltage;
}

void VCRInterface::receive_inverter_status_4(const CAN_message_t &can_msg)
{
    INV4_STATUS_t unpacked_msg;
    Unpack_INV4_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _inverter_status.error.RR = unpacked_msg.error;
    _inverter_status.dc_bus_voltage.RR = unpacked_msg.dc_bus_voltage;
}

void VCRInterface::receive_inverter_temperature_1(const CAN_message_t &can_msg)
{
    INV1_TEMPS_t unpacked_msg;
    Unpack_INV1_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _temps.motor_temps.FL = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);
    _temps.inverter_temps.FL = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
}

void VCRInterface::receive_inverter_temperature_2(const CAN_message_t &can_msg)
{
    INV2_TEMPS_t unpacked_msg;
    Unpack_INV2_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _temps.motor_temps.FR = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);
    _temps.inverter_temps.FR = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
}

void VCRInterface::receive_inverter_temperature_3(const CAN_message_t &can_msg)
{
    INV3_TEMPS_t unpacked_msg;
    Unpack_INV3_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _temps.motor_temps.RL = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);
    _temps.inverter_temps.RL = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
}

void VCRInterface::receive_inverter_temperature_4(const CAN_message_t &can_msg)
{
    INV4_TEMPS_t unpacked_msg;
    Unpack_INV4_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _temps.motor_temps.RR = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);
    _temps.inverter_temps.RR = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
}