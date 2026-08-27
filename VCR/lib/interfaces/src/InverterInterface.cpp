#include "InverterInterface.h"
#include "VCRCANInterfaceImpl.h"


/* ---------- Receiving Callbacks ---------- */

void InverterInterface::receive_GENERAL_CONTROL(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    HV500_TargetIq_t unpacked_msg;
    Unpack_INV1_STATUS_GENERAL_CONTROL_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_GENERAL_CONTROL_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id);

    auto& msg = _feedback_data.general_control_msg;
    msg.control_mode = static_cast<DTIControlMode_e>(unpacked_msg.control_mode);
    msg.target_iq_apk = unpacked_msg.target_iq_apk_ro / static_cast<float>(_dti_params.scales.iq_scale);
    msg.motor_position_deg = unpacked_msg.motor_position_deg_ro / static_cast<float>(_dti_params.scales.motor_position_scale);
    msg.is_motor_stationary = unpacked_msg.is_motor_stationary;

    _connected = true;
    _last_recv_millis = curr_millis;
}

void InverterInterface::receive_GENERAL_ELEC(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_GENERAL_ELEC_t unpacked_msg;
    Unpack_INV1_STATUS_GENERAL_ELEC_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_GENERAL_ELEC_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.general_elec_msg;
    msg.erpm = unpacked_msg.erpm;
    msg.duty_cycle_percent = unpacked_msg.duty_cycle_percent_ro / static_cast<float>(_dti_params.scales.duty_cycle_scale);
    msg.input_voltage = unpacked_msg.input_voltage;

    _connected = true;
    _last_recv_millis = curr_millis;
}

void InverterInterface::receive_ACTIVE_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_ACTIVE_CURRENT_t unpacked_msg;
    Unpack_INV1_STATUS_ACTIVE_CURRENT_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_ACTIVE_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.active_current_msg;
    msg.active_ac_current_apk = unpacked_msg.active_ac_current_apk_ro / static_cast<float>(_dti_params.scales.active_ac_current_scale);
    msg.active_dc_current_amp = unpacked_msg.active_dc_current_amp_ro / static_cast<float>(_dti_params.scales.active_dc_current_scale);

    _connected = true;
    _last_recv_millis = curr_millis;
}

void InverterInterface::receive_TEMP_AND_FAULT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_TEMP_AND_FAULT_t unpacked_msg;
    Unpack_INV1_STATUS_TEMP_AND_FAULT_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_TEMP_AND_FAULT_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.temp_and_fault_msg;
    msg.controller_temp_c = unpacked_msg.controller_temp_c_ro / static_cast<float>(_dti_params.scales.controller_temp_scale);
    msg.motor_temp_c = unpacked_msg.motor_temp_c_ro / static_cast<float>(_dti_params.scales.motor_temp_scale);
    msg.fault_code = static_cast<DTIFaultCode_e>(unpacked_msg.fault_code);

    _connected = true;
    _last_recv_millis = curr_millis;
}

void InverterInterface::receive_FOC_CURRENTS(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_FOC_CURRENTS_t unpacked_msg;
    Unpack_INV1_STATUS_FOC_CURRENTS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_FOC_CURRENTS_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.foc_current_msg;
    msg.id_apk = unpacked_msg.id_apk_ro / static_cast<float>(_dti_params.scales.foc_id_scale);
    msg.iq_apk = unpacked_msg.iq_apk_ro / static_cast<float>(_dti_params.scales.foc_iq_scale);

    _connected = true;
    _last_recv_millis = curr_millis;
}

void InverterInterface::receive_GENERAL_IO(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_GENERAL_IO_t unpacked_msg;
    Unpack_INV1_STATUS_GENERAL_IO_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_GENERAL_IO_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.general_io_msg;
    msg.throttle_signal_percent = unpacked_msg.throttle_signal_percent;
    msg.brake_signal_percent = unpacked_msg.brake_signal_percent;
    msg.is_digital_input_1_active = unpacked_msg.is_digital_input_1_active;
    msg.is_digital_input_2_active = unpacked_msg.is_digital_input_2_active;
    msg.is_digital_input_3_active = unpacked_msg.is_digital_input_3_active;
    msg.is_digital_input_4_active = unpacked_msg.is_digital_input_4_active;
    msg.is_digital_output_1_active = unpacked_msg.is_digital_output_1_active;
    msg.is_digital_output_2_active = unpacked_msg.is_digital_output_2_active;
    msg.is_digital_output_3_active = unpacked_msg.is_digital_output_3_active;
    msg.is_digital_output_4_active = unpacked_msg.is_digital_output_4_active;
    msg.is_drive_enabled = unpacked_msg.is_drive_enabled;
    msg.is_capacitor_temp_limit_active = unpacked_msg.is_capacitor_temp_limit_active;
    msg.is_dc_current_limit_active = unpacked_msg.is_dc_current_limit_active;
    msg.is_drive_enable_limit_active = unpacked_msg.is_drive_enable_limit_active;
    msg.is_igbt_accel_temp_limit_active = unpacked_msg.is_igbt_accel_temp_limit_active;
    msg.is_igbt_temp_limit_active = unpacked_msg.is_igbt_temp_limit_active;
    msg.is_input_voltage_limit_active = unpacked_msg.is_input_voltage_limit_active;
    msg.is_motor_accel_temp_limit_active = unpacked_msg.is_motor_accel_temp_limit_active;
    msg.is_motor_temp_limit_active = unpacked_msg.is_motor_temp_limit_active;
    msg.is_rpm_min_limit_active = unpacked_msg.is_rpm_min_limit_active;
    msg.is_rpm_max_limit_active = unpacked_msg.is_rpm_max_limit_active;
    msg.is_power_limit_active = unpacked_msg.is_power_limit_active;
    msg.can_map_version = unpacked_msg.can_map_version;

    _connected = true;
    _last_recv_millis = curr_millis;
}

void InverterInterface::receive_AC_CONFIG_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_AC_CONFIG_CURRENT_t unpacked_msg;
    Unpack_INV1_STATUS_AC_CONFIG_CURRENT_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_AC_CONFIG_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.ac_config_current_msg;
    msg.max_ac_current_apk = unpacked_msg.max_ac_current_apk_ro / static_cast<float>(_dti_params.scales.max_ac_current_scale);
    msg.available_max_ac_current_apk = unpacked_msg.available_max_ac_current_apk_ro / static_cast<float>(_dti_params.scales.available_max_ac_current_scale);
    msg.min_ac_current_apk = unpacked_msg.min_ac_current_apk_ro / static_cast<float>(_dti_params.scales.min_ac_current_scale);
    msg.available_min_ac_current_apk = unpacked_msg.available_min_ac_current_apk_ro / static_cast<float>(_dti_params.scales.available_min_ac_current_scale);

    _connected = true;
    _last_recv_millis = curr_millis;
}

void InverterInterface::receive_DC_CONFIG_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_DC_CONFIG_CURRENT_t unpacked_msg;
    Unpack_INV1_STATUS_DC_CONFIG_CURRENT_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_DC_CONFIG_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.dc_config_current_msg;
    msg.max_dc_current_amp = unpacked_msg.max_dc_current_amp_ro / static_cast<float>(_dti_params.scales.max_dc_current_scale);
    msg.available_max_dc_current_amp = unpacked_msg.available_max_dc_current_amp_ro / static_cast<float>(_dti_params.scales.available_max_dc_current_scale);
    msg.min_dc_current_amp = unpacked_msg.min_dc_current_amp_ro / static_cast<float>(_dti_params.scales.min_dc_current_scale);
    msg.available_min_dc_current_amp = unpacked_msg.available_min_dc_current_amp_ro / static_cast<float>(_dti_params.scales.available_min_dc_current_scale);

    _connected = true;
    _last_recv_millis = curr_millis;
}


/* ---------- Sending ---------- */

void InverterInterface::send_AC_CURRENT(float target_ac_current_apk)
{
    target_ac_current_apk = std::clamp(target_ac_current_apk, -850.0f, 850.0f);   // signed: sign = torque direction

    INV1_SET_AC_CURRENT_t msg_out;
    msg_out.target_ac_current_apk_ro = HYTECH_target_ac_current_apk_ro_toS(target_ac_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_AC_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_AC_CURRENT)
    );
}

void InverterInterface::send_BRAKE_CURRENT(float target_brake_current_apk)
{
    target_brake_current_apk = std::clamp(target_brake_current_apk, 0.0f, 850.0f);

    INV1_SET_BRAKE_CURRENT_t msg_out;
    msg_out.target_brake_current_apk_ro = HYTECH_target_brake_current_apk_ro_toS(target_brake_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_BRAKE_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_BRAKE_CURRENT)
    );
}

void InverterInterface::send_ERPM(float target_erpm)
{
    target_erpm = std::clamp(target_erpm, -100000.0f, 100000.0f);

    INV1_SET_ERPM_t msg_out;
    msg_out.target_erpm = target_erpm;

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_ERPM_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_ERPM)
    );
}

void InverterInterface::send_MOTOR_POSITION(float target_motor_position_deg)
{
    target_motor_position_deg = std::clamp(target_motor_position_deg, 0.0f, 359.0f);

    INV1_SET_MOTOR_POSITION_t msg_out;
    msg_out.target_motor_position_deg_ro = HYTECH_target_motor_position_deg_ro_toS(target_motor_position_deg);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MOTOR_POSITION_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_MOTOR_POSITION)
    );
}

void InverterInterface::send_REL_AC_CURRENT(float target_ac_current_percent)
{
    target_ac_current_percent = std::clamp(target_ac_current_percent, -100.0f, 100.0f);

    INV1_SET_REL_AC_CURRENT_t msg_out;
    msg_out.target_ac_current_percent_ro = HYTECH_target_ac_current_percent_ro_toS(target_ac_current_percent);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_REL_AC_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_RELATIVE_AC_CURRENT)
    );
}

void InverterInterface::send_REL_AC_BRAKE_CURRENT(float target_ac_brake_current_percent)
{
    target_ac_brake_current_percent = std::clamp(target_ac_brake_current_percent, 0.0f, 100.0f);

    INV1_SET_REL_AC_BRAKE_CURRENT_t msg_out;
    msg_out.target_ac_brake_current_percent_ro = HYTECH_target_ac_brake_current_percent_ro_toS(target_ac_brake_current_percent);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_REL_AC_BRAKE_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_RELATIVE_AC_BRAKE_CURRENT)
    );
}

void InverterInterface::send_DIGITAL_OUTPUTS()
{
    // TODO: unimplemented — digital output usage/wiring not yet determined.
    // Once known, populate INV1_SET_DIGITAL_OUTPUTS_t from _set_commands.digital_output_msg
    // (or take the 4 bools as parameters, matching whichever call pattern is decided).
}

void InverterInterface::send_MAX_AC_CURRENT(float target_max_ac_current_apk)
{
    target_max_ac_current_apk = std::clamp(target_max_ac_current_apk, 0.0f, 850.0f);

    INV1_SET_MAX_AC_CURRENT_t msg_out;
    msg_out.max_ac_current_apk_ro = HYTECH_max_ac_current_apk_ro_toS(target_max_ac_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_AC_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_MAX_AC_CURRENT)
    );
}

void InverterInterface::send_MAX_AC_BRAKE_CURRENT(float target_max_ac_brake_current_apk)
{
    target_max_ac_brake_current_apk = std::clamp(target_max_ac_brake_current_apk, 0.0f, 850.0f);

    INV1_SET_MAX_AC_BRAKE_CURRENT_t msg_out;
    msg_out.max_ac_brake_current_apk_ro = HYTECH_max_ac_brake_current_apk_ro_toS(target_max_ac_brake_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_AC_BRAKE_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_MAX_AC_BRAKE_CURRENT)
    );
}

void InverterInterface::send_MAX_DC_CURRENT(float target_max_dc_current_apk)
{
    target_max_dc_current_apk = std::clamp(target_max_dc_current_apk, 0.0f, 850.0f);

    INV1_SET_MAX_DC_CURRENT_t msg_out;
    msg_out.max_dc_current_amp_ro = HYTECH_max_dc_current_amp_ro_toS(target_max_dc_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_DC_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_MAX_DC_CURRENT)
    );
}

void InverterInterface::send_MAX_DC_BRAKE_CURRENT(float target_max_dc_brake_current_apk)
{
    target_max_dc_brake_current_apk = std::clamp(target_max_dc_brake_current_apk, 0.0f, 850.0f);

    INV1_SET_MAX_DC_BRAKE_CURRENT_t msg_out;
    msg_out.max_dc_brake_current_amp_ro = HYTECH_max_dc_brake_current_amp_ro_toS(target_max_dc_brake_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_DC_BRAKE_CURRENT_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_MAX_DC_BRAKE_CURRENT)
    );
}

void InverterInterface::send_DRIVE_ENABLE()
{
    INV1_SET_DRIVE_ENABLE_t msg_out;
    msg_out.is_drive_enabled = _enable_requested;

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_DRIVE_ENABLE_hytech,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _pack_dti_can_id(dti_command_packet_ids::SET_DRIVE_ENABLE)
    );
}


/* ---------- InverterFuncts_s-facing API ---------- */

void InverterInterface::set_torque(float torque_nm)
{
    float current_apk = _torque_to_current_apk(torque_nm);

    if (torque_nm >= 0.0f)
    {
        send_AC_CURRENT(current_apk);
    }
    else
    {
        send_BRAKE_CURRENT(current_apk);   // magnitude only, sign handled by which command you send
    }
}

void InverterInterface::set_idle()
{
    send_AC_CURRENT(0.0f);
}

void InverterInterface::request_enable(bool enable)
{
    _enable_requested = enable;
    send_DRIVE_ENABLE();
}

void InverterInterface::request_error_reset()
{
    // TODO: DTI's fault-clearing mechanism is still unconfirmed — no reset
    // command has been identified in the manual so far. No-op until confirmed;
    // do not let CLEARING_ERRORS state logic depend on this actually doing
    // something until it's resolved.
}

InverterStatus_s InverterInterface::get_status() const
{
    InverterStatus_s status{};
    status.is_inverter_connected = _connected;
    status.is_fault_code_present = (_feedback_data.temp_and_fault_msg.fault_code != DTIFaultCode_e::NO_FAULTS);
    status.is_hv_present = _feedback_data.general_elec_msg.input_voltage > _dti_params.minimum_hv_voltage;
    status.dc_bus_voltage = _feedback_data.general_elec_msg.input_voltage;
    status.last_recv_millis = _last_recv_millis;
    return status;
}

MotorMechanics_s InverterInterface::get_motor_mechanics() const
{
    float id_actual = _feedback_data.foc_current_msg.id_apk;
    float iq_actual = _feedback_data.foc_current_msg.iq_apk;

    float lambda_eff = _dti_params.lambda_pm_wb +
                    (_dti_params.ld_henries - _dti_params.lq_henries) *
                    id_actual;

    MotorMechanics_s mm{};
    mm.actual_speed = static_cast<float>(_feedback_data.general_elec_msg.erpm) / static_cast<float>(_dti_params.num_pole_pairs);
    mm.actual_torque = 1.5f * _dti_params.num_pole_pairs * lambda_eff * iq_actual;
    mm.actual_power = _feedback_data.general_elec_msg.input_voltage * _feedback_data.active_current_msg.active_dc_current_amp;
    mm.last_recv_millis = _last_recv_millis;

    return mm;
}


/* ---------- DTI-specific diagnostics ---------- */

DTIFaultCode_e InverterInterface::get_fault_code() const
{
    return _feedback_data.temp_and_fault_msg.fault_code;
}

const StatusGeneralIOMsg_s& InverterInterface::get_io_status() const
{
    return _feedback_data.general_io_msg;
}

const StatusGeneralControlMsg_s& InverterInterface::get_control_status() const
{
    return _feedback_data.general_control_msg;
}

const DTIStatusMessages_s& InverterInterface::get_all_inverter_data() const
{
    return _feedback_data;
}


/* ---------- Private helpers ---------- */

float InverterInterface::_torque_to_current_apk(float torque_nm) const
{
    float id_act = _feedback_data.foc_current_msg.id_apk;   // measured, from 0x23

    // Formula based on recommended equation in documentation
    float lambda_effective = _dti_params.lambda_pm_wb +
                            (_dti_params.ld_henries - _dti_params.lq_henries) *
                            id_act;

    float iq_ff = (2.0f * torque_nm) / (3.0f * _dti_params.num_pole_pairs * lambda_effective);

    // This is i sub s, not "is"
    float is_magnitude = std::sqrt(id_act * id_act + iq_ff * iq_ff);

    return is_magnitude;
}

uint16_t InverterInterface::_pack_dti_can_id(uint8_t packet_id) const
{
    return (static_cast<uint16_t>(packet_id) << 5) | (_node_id & 0x1F);
}