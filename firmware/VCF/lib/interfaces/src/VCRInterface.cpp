#include "VCRInterface.h"

void VCRInterface::receive_dash_control_data(const CAN_message_t &can_msg)
{
    DASHBOARD_BUZZER_CONTROL_t unpacked_msg;
    Unpack_DASHBOARD_BUZZER_CONTROL_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT

    if (unpacked_msg.dash_buzzer_flag)
    {
        BuzzerController::getInstance().activate(millis());
    }

    _is_in_pedals_calibration_state = unpacked_msg.in_pedal_calibration_state;
    _is_in_steering_calibration_state = unpacked_msg.in_steering_calibration_state;

    if (unpacked_msg.torque_limit_enum_value < ((int) TorqueLimit_e::TCMUX_NUM_TORQUE_LIMITS)) // check for validity
    {
        _torque_limit = (TorqueLimit_e) unpacked_msg.torque_limit_enum_value;
    }
}

void VCRInterface::receive_car_states_data(const CAN_message_t &can_msg)
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
    _inv_error_status.error.FL = unpacked_msg.error;
    _bus_voltages.voltage.FL = unpacked_msg.dc_bus_voltage;
}

void VCRInterface::receive_inverter_status_2(const CAN_message_t &can_msg)
{
    INV2_STATUS_t unpacked_msg;
    Unpack_INV2_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _inv_error_status.error.FR = unpacked_msg.error;
    _bus_voltages.voltage.FR = unpacked_msg.dc_bus_voltage;
}

void VCRInterface::receive_inverter_status_3(const CAN_message_t &can_msg)
{
    INV3_STATUS_t unpacked_msg;
    Unpack_INV3_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _inv_error_status.error.RL = unpacked_msg.error;
    _bus_voltages.voltage.RL = unpacked_msg.dc_bus_voltage;
}

void VCRInterface::receive_inverter_status_4(const CAN_message_t &can_msg)
{
    INV4_STATUS_t unpacked_msg;
    Unpack_INV4_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _inv_error_status.error.RR = unpacked_msg.error;
    _bus_voltages.voltage.RR = unpacked_msg.dc_bus_voltage;
}

bool VCRInterface::get_inverter_error()
{
    return _inv_error_status.error.FL || _inv_error_status.error.FR || _inv_error_status.error.RL || _inv_error_status.error.RR;
}