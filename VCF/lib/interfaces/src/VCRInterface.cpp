#include "VCRInterface.hpp"


void VCRInterface::receiveDashboardControlData(const CAN_message_t& can_msg)
{
    DASHBOARD_BUZZER_CONTROL_t unpacked_msg;
    Unpack_DASHBOARD_BUZZER_CONTROL_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT

    if (unpacked_msg.dash_buzzer_flag)
    {
        BuzzerControllerInstance::instance().activate(millis());
    }

    _is_in_pedals_calibration_state = unpacked_msg.in_pedal_calibration_state;
    _is_in_steering_calibration_state = unpacked_msg.in_steering_calibration_state;

    if (unpacked_msg.torque_limit_enum_value < ((int)TorqueLimit_e::TCMUX_NUM_TORQUE_LIMITS)) // check for validity
    {
        _torque_limit = (TorqueLimit_e)unpacked_msg.torque_limit_enum_value;
    }
}

void VCRInterface::recieveCarStatesCANMsg(const CAN_message_t& can_msg)
{
    CAR_STATES_t unpacked_msg;
    Unpack_CAR_STATES_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
    _vehicle_state_value = static_cast<VehicleState_e>(unpacked_msg.vehicle_state);
    _drivetrain_state_value = static_cast<DrivetrainState_e>(unpacked_msg.drivetrain_state);
    _is_db_in_ctrl = unpacked_msg.drivebrain_in_control;
}

void VCRInterface::receiveInverterFaultCANMsg(const CAN_message_t& can_msg)
{
    switch (can_msg.id)
    {
    case INV1_STATUS_TEMP_AND_FAULT_CANID:
    {
        INV1_STATUS_TEMP_AND_FAULT_t unpacked_msg;
        Unpack_INV1_STATUS_TEMP_AND_FAULT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _inv_error_status.error.FL = (unpacked_msg.fault_code != 0);
        break;
    }
    case INV2_STATUS_TEMP_AND_FAULT_CANID:
    {
        INV2_STATUS_TEMP_AND_FAULT_t unpacked_msg;
        Unpack_INV2_STATUS_TEMP_AND_FAULT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _inv_error_status.error.FR = (unpacked_msg.fault_code != 0);
        break;
    }
    case INV3_STATUS_TEMP_AND_FAULT_CANID:
    {
        INV3_STATUS_TEMP_AND_FAULT_t unpacked_msg;
        Unpack_INV3_STATUS_TEMP_AND_FAULT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _inv_error_status.error.RL = (unpacked_msg.fault_code != 0);
        break;
    }
    case INV4_STATUS_TEMP_AND_FAULT_CANID:
    {
        INV4_STATUS_TEMP_AND_FAULT_t unpacked_msg;
        Unpack_INV4_STATUS_TEMP_AND_FAULT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _inv_error_status.error.RR = (unpacked_msg.fault_code != 0);
        break;
    }
    default:
        break;
    }
}

void VCRInterface::receiveInverterElecCANMsg(const CAN_message_t& can_msg)
{
    switch (can_msg.id)
    {
    case INV1_STATUS_GENERAL_ELEC_CANID:
    {
        INV1_STATUS_GENERAL_ELEC_t unpacked_msg;
        Unpack_INV1_STATUS_GENERAL_ELEC_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _bus_voltages.voltage.FL = unpacked_msg.input_voltage;
        break;
    }
    case INV2_STATUS_GENERAL_ELEC_CANID:
    {
        INV2_STATUS_GENERAL_ELEC_t unpacked_msg;
        Unpack_INV2_STATUS_GENERAL_ELEC_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _bus_voltages.voltage.FR = unpacked_msg.input_voltage;
        break;
    }
    case INV3_STATUS_GENERAL_ELEC_CANID:
    {
        INV3_STATUS_GENERAL_ELEC_t unpacked_msg;
        Unpack_INV3_STATUS_GENERAL_ELEC_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _bus_voltages.voltage.RL = unpacked_msg.input_voltage;
        break;
    }
    case INV4_STATUS_GENERAL_ELEC_CANID:
    {
        INV4_STATUS_GENERAL_ELEC_t unpacked_msg;
        Unpack_INV4_STATUS_GENERAL_ELEC_ht_can(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT
        _bus_voltages.voltage.RR = unpacked_msg.input_voltage;
        break;
    }
    default:
        break;
    }
}

bool VCRInterface::isInverterErrored()
{
    return _inv_error_status.error.FL || _inv_error_status.error.FR || _inv_error_status.error.RL || _inv_error_status.error.RR;
}
