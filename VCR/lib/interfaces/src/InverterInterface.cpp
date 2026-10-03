#include "InverterInterface.hpp"
#include "VCRCANInterfaceImpl.hpp"

/**
 * @note All scaling for signals is defined in the datasheet; we will use the coderdbc API for receiving and sending the
 *       correctly scaled signal.
 * @note ro -> raw output
 *       When the datasheet specifies the scale is 10 for example: raw = physical value on the wire × 10
 *       PCAN's "Factor" field works the opposite direction: physical value on the wire = raw × Factor
 *       Which is why in PCAN we say factor is 0.1.
*/


/* ---------- Receiving ---------- */

void InverterInterface::receive_GENERAL_CONTROL(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_GENERAL_CONTROL_t unpacked_msg{};
    Unpack_INV1_STATUS_GENERAL_CONTROL_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_GENERAL_CONTROL_ht_can,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id);

    auto& msg = _feedback_data.general_control_msg;
    msg.control_mode = static_cast<DTIControlMode_e>(unpacked_msg.control_mode);
    msg.target_iq_apk = HT_CAN_target_iq_apk_ro_fromS(unpacked_msg.target_iq_apk_ro);
    msg.motor_position_deg = HT_CAN_motor_position_deg_ro_fromS(unpacked_msg.motor_position_deg_ro);
    msg.is_motor_stationary = unpacked_msg.is_motor_stationary;

    _markReceived(curr_millis);
}

void InverterInterface::receive_GENERAL_ELEC(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_GENERAL_ELEC_t unpacked_msg{};
    Unpack_INV1_STATUS_GENERAL_ELEC_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_GENERAL_ELEC_ht_can,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.general_elec_msg;
    msg.erpm = unpacked_msg.erpm;
    msg.duty_cycle_percent = HT_CAN_duty_cycle_percent_ro_fromS(unpacked_msg.duty_cycle_percent_ro);
    msg.input_voltage = unpacked_msg.input_voltage;

    _markReceived(curr_millis);
}

void InverterInterface::receive_ACTIVE_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_ACTIVE_CURRENT_t unpacked_msg{};
    Unpack_INV1_STATUS_ACTIVE_CURRENT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_ACTIVE_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.active_current_msg;
    msg.active_ac_current_apk = HT_CAN_active_ac_current_apk_ro_fromS(unpacked_msg.active_ac_current_apk_ro);
    msg.active_dc_current_amp = HT_CAN_active_dc_current_amp_ro_fromS(unpacked_msg.active_dc_current_amp_ro);

    _markReceived(curr_millis);
}

void InverterInterface::receive_TEMP_AND_FAULT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_TEMP_AND_FAULT_t unpacked_msg{};
    Unpack_INV1_STATUS_TEMP_AND_FAULT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_TEMP_AND_FAULT_ht_can,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.temp_and_fault_msg;
    msg.controller_temp_c = HT_CAN_controller_temp_c_ro_fromS(unpacked_msg.controller_temp_c_ro);
    msg.motor_temp_c = HT_CAN_motor_temp_c_ro_fromS(unpacked_msg.motor_temp_c_ro);
    msg.fault_code = static_cast<DTIFaultCode_e>(unpacked_msg.fault_code);

    _markReceived(curr_millis);
}

void InverterInterface::receive_FOC_CURRENTS(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_FOC_CURRENTS_t unpacked_msg{};
    Unpack_INV1_STATUS_FOC_CURRENTS_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_FOC_CURRENTS_ht_can,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.foc_current_msg;
    msg.id_apk = HT_CAN_id_apk_ro_fromS(unpacked_msg.id_apk_ro);
    msg.iq_apk = HT_CAN_iq_apk_ro_fromS(unpacked_msg.iq_apk_ro);

    _markReceived(curr_millis);
}

void InverterInterface::receive_GENERAL_IO(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_GENERAL_IO_t unpacked_msg{};
    Unpack_INV1_STATUS_GENERAL_IO_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_GENERAL_IO_ht_can,
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
    msg.can_map_version = static_cast<float>(unpacked_msg.can_map_version) / 10.0f;   // raw 25 -> 2.5

    _markReceived(curr_millis);
}

void InverterInterface::receive_AC_CONFIG_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_AC_CONFIG_CURRENT_t unpacked_msg{};
    Unpack_INV1_STATUS_AC_CONFIG_CURRENT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_AC_CONFIG_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.ac_config_current_msg;
    msg.max_ac_current_apk = HT_CAN_max_ac_current_apk_ro_fromS(unpacked_msg.max_ac_current_apk_ro);
    msg.available_max_ac_current_apk = HT_CAN_available_max_ac_current_apk_ro_fromS(unpacked_msg.available_max_ac_current_apk_ro);
    msg.min_ac_current_apk = HT_CAN_min_ac_current_apk_ro_fromS(unpacked_msg.min_ac_current_apk_ro);
    msg.available_min_ac_current_apk = HT_CAN_available_min_ac_current_apk_ro_fromS(unpacked_msg.available_min_ac_current_apk_ro);

    _markReceived(curr_millis);
}

void InverterInterface::receive_DC_CONFIG_CURRENT(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    INV1_STATUS_DC_CONFIG_CURRENT_t unpacked_msg{};
    Unpack_INV1_STATUS_DC_CONFIG_CURRENT_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueue_msg(&unpacked_msg,
                        &Pack_INV1_STATUS_DC_CONFIG_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
                        can_msg.id
    );

    auto& msg = _feedback_data.dc_config_current_msg;
    msg.max_dc_current_amp = HT_CAN_max_dc_current_amp_ro_fromS(unpacked_msg.max_dc_current_amp_ro);
    msg.available_max_dc_current_amp = HT_CAN_available_max_dc_current_amp_ro_fromS(unpacked_msg.available_max_dc_current_amp_ro);
    msg.min_dc_current_amp = HT_CAN_min_dc_current_amp_ro_fromS(unpacked_msg.min_dc_current_amp_ro);
    msg.available_min_dc_current_amp = HT_CAN_available_min_dc_current_amp_ro_fromS(unpacked_msg.available_min_dc_current_amp_ro);

    _markReceived(curr_millis);
}


/* ---------- Sending ---------- */

void InverterInterface::send_AC_CURRENT(float target_ac_current_apk)
{
    target_ac_current_apk = std::isfinite(target_ac_current_apk) ? std::clamp(target_ac_current_apk, -850.0f, 850.0f) : 0.0f;   // signed: sign = torque direction

    INV1_SET_AC_CURRENT_t msg_out{};
    msg_out.target_ac_current_apk_ro = HT_CAN_target_ac_current_apk_ro_toS(target_ac_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_AC_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_AC_CURRENT)
    );
}

void InverterInterface::send_BRAKE_CURRENT(float target_brake_current_apk)
{
    target_brake_current_apk = std::isfinite(target_brake_current_apk) ? (target_brake_current_apk, 0.0f, 850.0f) : 0.0f;

    INV1_SET_BRAKE_CURRENT_t msg_out{};
    msg_out.target_brake_current_apk_ro = HT_CAN_target_brake_current_apk_ro_toS(target_brake_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_BRAKE_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_BRAKE_CURRENT)
    );
}

void InverterInterface::send_ERPM(float target_erpm)
{
    target_erpm = std::isfinite(target_erpm) ? (target_erpm, -100000.0f, 100000.0f) : 0.0f;

    INV1_SET_ERPM_t msg_out{};
    msg_out.target_erpm = target_erpm;

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_ERPM_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_ERPM)
    );
}

void InverterInterface::send_MOTOR_POSITION(float target_motor_position_deg)
{
    target_motor_position_deg = std::isfinite(target_motor_position_deg) ? (target_motor_position_deg, 0.0f, 359.9f) : 0.0f;

    INV1_SET_MOTOR_POSITION_t msg_out{};
    msg_out.target_motor_position_deg_ro = HT_CAN_target_motor_position_deg_ro_toS(target_motor_position_deg);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MOTOR_POSITION_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_MOTOR_POSITION)
    );
}

void InverterInterface::send_REL_AC_CURRENT(float target_ac_current_percent)
{
    target_ac_current_percent = std::isfinite(target_ac_current_percent) ? (target_ac_current_percent, -100.0f, 100.0f) : 0.0f;

    INV1_SET_REL_AC_CURRENT_t msg_out{};
    msg_out.target_ac_current_percent_ro = HT_CAN_target_ac_current_percent_ro_toS(target_ac_current_percent);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_REL_AC_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_RELATIVE_AC_CURRENT)
    );
}

void InverterInterface::send_REL_AC_BRAKE_CURRENT(float target_ac_brake_current_percent)
{
    target_ac_brake_current_percent = std::isfinite(target_ac_brake_current_percent) ? (target_ac_brake_current_percent, 0.0f, 100.0f) : 0.0f;

    INV1_SET_REL_AC_BRAKE_CURRENT_t msg_out{};
    msg_out.target_ac_brake_current_percent_ro = HT_CAN_target_ac_brake_current_percent_ro_toS(target_ac_brake_current_percent);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_REL_AC_BRAKE_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_RELATIVE_AC_BRAKE_CURRENT)
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
    target_max_ac_current_apk = std::isfinite(target_max_ac_current_apk) ? (target_max_ac_current_apk, 0.0f, 850.0f) : 0.0f;

    INV1_SET_MAX_AC_CURRENT_t msg_out{};
    // Uses the command signal's own scaling macro (was the 0x25 status signal's macro)
    msg_out.target_max_ac_current_apk_ro = HT_CAN_target_max_ac_current_apk_ro_toS(target_max_ac_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_AC_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_MAX_AC_CURRENT)
    );
}

void InverterInterface::send_MAX_AC_BRAKE_CURRENT(float target_max_ac_brake_current_apk)
{
    target_max_ac_brake_current_apk = std::isfinite(target_max_ac_brake_current_apk) ? (target_max_ac_brake_current_apk, -850.0f, 0.0f) : 0.0f;

    INV1_SET_MAX_AC_BRAKE_CURRENT_t msg_out{};
    msg_out.target_max_ac_brake_current_apk_ro = HT_CAN_target_max_ac_brake_current_apk_ro_toS(target_max_ac_brake_current_apk);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_AC_BRAKE_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_MAX_AC_BRAKE_CURRENT)
    );
}

void InverterInterface::send_MAX_DC_CURRENT(float target_max_dc_current_amp)
{
    target_max_dc_current_amp = std::isfinite(target_max_dc_current_amp) ? (target_max_dc_current_amp, 0.0f, 850.0f) : 0.0f;

    INV1_SET_MAX_DC_CURRENT_t msg_out{};
    // Uses the command signal's own scaling macro (was the 0x26 status signal's macro)
    msg_out.target_max_dc_current_amp_ro = HT_CAN_target_max_dc_current_amp_ro_toS(target_max_dc_current_amp);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_DC_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_MAX_DC_CURRENT)
    );
}

void InverterInterface::send_MAX_DC_BRAKE_CURRENT(float target_max_dc_brake_current_amp)
{
    target_max_dc_brake_current_amp = std::isfinite(target_max_dc_brake_current_amp) ? (target_max_dc_brake_current_amp, -850.0f, 0.0f) : 0.0f;

    INV1_SET_MAX_DC_BRAKE_CURRENT_t msg_out{};
    msg_out.target_max_dc_brake_current_amp_ro = HT_CAN_target_max_dc_brake_current_amp_ro_toS(target_max_dc_brake_current_amp);

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_MAX_DC_BRAKE_CURRENT_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_MAX_DC_BRAKE_CURRENT)
    );
}

void InverterInterface::send_DRIVE_ENABLE()
{
    INV1_SET_DRIVE_ENABLE_t msg_out{};
    msg_out.drive_enable_requested = _enable_requested;

    CAN_util::enqueue_msg(&msg_out,
                        &Pack_INV1_SET_DRIVE_ENABLE_ht_can,
                        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
                        _packCANID(dti_command_packet_ids::SET_DRIVE_ENABLE)
    );
}

void InverterInterface::sendControlCommands(DrivetrainControlMode_e mode)
{
    send_DRIVE_ENABLE();

    if (!_enable_requested)
    {
        send_AC_CURRENT(0.0f);
        return;
    }

    switch (mode)
    {
        case DrivetrainControlMode_e::TORQUE:
        {
            if (_control_inputs.pending_ac_current_apk > 0.0f)
            {
                send_AC_CURRENT(_control_inputs.pending_ac_current_apk);
            }
            else
            {
                send_BRAKE_CURRENT(_control_inputs.pending_ac_brake_current_apk);
            }
            break;
        }
        case DrivetrainControlMode_e::SPEED:
        {
            send_ERPM(_control_inputs.pending_speed_erpm);
            break;
        }
        default:
        {
            send_AC_CURRENT(0.0f);   // unknown mode: command no torque
            break;
        }
    }
}


/* ---------- InverterInterfaceFuncts_s-facing API ---------- */

void InverterInterface::setMotorTorque(float requested_torque_nm)
{
    float total_ac_current_mag = _convertTorqueToCurrent(requested_torque_nm);

    // Exactly one of the two is nonzero, so sendControlCommands() never sees a stale value from the other direction
    if (total_ac_current_mag >= 0.0f)
    {
        _control_inputs.pending_ac_current_apk = total_ac_current_mag;
        _control_inputs.pending_ac_brake_current_apk = 0.0f;
    }
    else
    {
        _control_inputs.pending_ac_current_apk = 0.0f;
        _control_inputs.pending_ac_brake_current_apk = -total_ac_current_mag;
    }
}

void InverterInterface::setMotorSpeed(float requested_speed_rpm)
{
    _control_inputs.pending_speed_erpm = _convertRPMToERPM(requested_speed_rpm);
}

void InverterInterface::setMotorIdle()
{
    _control_inputs.pending_ac_current_apk = 0.0f;
    _control_inputs.pending_ac_brake_current_apk = 0.0f;
    _control_inputs.pending_speed_erpm = 0.0f;
}

void InverterInterface::requestEnable(bool enable)
{
    _enable_requested = enable;
}

bool InverterInterface::isReportedModeMatchingDTS(DrivetrainControlMode_e expected_mode) const
{
    DTIControlMode_e reported_mode = _feedback_data.general_control_msg.control_mode;

    switch (expected_mode)
    {
        case DrivetrainControlMode_e::TORQUE:
        {
            return (reported_mode == DTIControlMode_e::MODE_CURRENT) || (reported_mode == DTIControlMode_e::MODE_CURRENT_BRAKE);
        }
        case DrivetrainControlMode_e::SPEED:
        {
            return (reported_mode == DTIControlMode_e::MODE_SPEED);
        }
        default:
        {
            return false;
        }
    }
}

InverterStatus_s InverterInterface::getStatus(unsigned long curr_millis) const
{
    // Signed elapsed time: if a frame arrived after the caller sampled curr_millis (e.g. from the CAN ISR),
    // the unsigned difference would wrap to a huge value and read as a disconnect. Negative elapsed = just received.
    long elapsed_ms = static_cast<long>(curr_millis - _last_recv_millis);

    InverterStatus_s status{};
    status.is_inverter_connected = _has_received && (elapsed_ms < static_cast<long>(_dti_params.connection_timeout_ms));
    status.is_fault_code_present = (_feedback_data.temp_and_fault_msg.fault_code != DTIFaultCode_e::NO_FAULTS);
    status.is_hv_present = status.is_inverter_connected &&
                           (_feedback_data.general_elec_msg.input_voltage > _dti_params.minimum_hv_voltage);
    status.is_drive_enabled = _feedback_data.general_io_msg.is_drive_enabled;
    status.dc_bus_voltage = _feedback_data.general_elec_msg.input_voltage;
    status.last_recv_millis = _last_recv_millis;
    return status;
}

MotorMechanics_s InverterInterface::getMotorMechanics() const
{
    float id_actual = _feedback_data.foc_current_msg.id_apk;
    float iq_actual = _feedback_data.foc_current_msg.iq_apk;
    float torque_actual_nm = _convertCurrentToTorque(id_actual, iq_actual);

    MotorMechanics_s mm{};
    mm.actual_speed_rpm = static_cast<float>(_feedback_data.general_elec_msg.erpm) / static_cast<float>(_dti_params.num_pole_pairs);
    mm.actual_torque_nm = torque_actual_nm;
    // NOTE: this is DC electrical input power (V_dc * I_dc), not shaft power
    mm.actual_power_watts = _feedback_data.general_elec_msg.input_voltage * _feedback_data.active_current_msg.active_dc_current_amp;
    mm.last_recv_millis = _last_recv_millis;

    return mm;
}

InverterData_s InverterInterface::getTelemetryData() const
{
    InverterData_s data{};

    // 0x1F — General control
    data.control_mode = _feedback_data.general_control_msg.control_mode;
    data.target_iq_apk = _feedback_data.general_control_msg.target_iq_apk;
    data.motor_position_deg = _feedback_data.general_control_msg.motor_position_deg;
    data.is_motor_stationary = _feedback_data.general_control_msg.is_motor_stationary;

    // 0x20 — General elec
    data.erpm = _feedback_data.general_elec_msg.erpm;
    data.duty_cycle_percent = _feedback_data.general_elec_msg.duty_cycle_percent;
    data.input_voltage = _feedback_data.general_elec_msg.input_voltage;

    // 0x21 — Active current
    data.active_ac_current_apk = _feedback_data.active_current_msg.active_ac_current_apk;
    data.active_dc_current_amp = _feedback_data.active_current_msg.active_dc_current_amp;

    // 0x22 — Temp and fault
    data.controller_temp_c = _feedback_data.temp_and_fault_msg.controller_temp_c;
    data.motor_temp_c = _feedback_data.temp_and_fault_msg.motor_temp_c;
    data.fault_code = _feedback_data.temp_and_fault_msg.fault_code;

    // 0x23 — FOC currents
    data.iq_apk = _feedback_data.foc_current_msg.iq_apk;
    data.id_apk = _feedback_data.foc_current_msg.id_apk;

    // 0x24 — General IO (data fields only; limit-active flags excluded)
    data.throttle_signal_percent = _feedback_data.general_io_msg.throttle_signal_percent;
    data.brake_signal_percent = _feedback_data.general_io_msg.brake_signal_percent;
    data.is_drive_enabled = _feedback_data.general_io_msg.is_drive_enabled;
    data.can_map_version = _feedback_data.general_io_msg.can_map_version;

    return data;
}

InverterLimits_s InverterInterface::getLimitsData() const
{
    InverterLimits_s limits{};

    // 0x25 — AC current limits
    limits.max_ac_current_apk = _feedback_data.ac_config_current_msg.max_ac_current_apk;
    limits.available_max_ac_current_apk = _feedback_data.ac_config_current_msg.available_max_ac_current_apk;
    limits.min_ac_current_apk = _feedback_data.ac_config_current_msg.min_ac_current_apk;
    limits.available_min_ac_current_apk = _feedback_data.ac_config_current_msg.available_min_ac_current_apk;

    // 0x26 — DC current limits
    limits.max_dc_current_amp = _feedback_data.dc_config_current_msg.max_dc_current_amp;
    limits.available_max_dc_current_amp = _feedback_data.dc_config_current_msg.available_max_dc_current_amp;
    limits.min_dc_current_amp = _feedback_data.dc_config_current_msg.min_dc_current_amp;
    limits.available_min_dc_current_amp = _feedback_data.dc_config_current_msg.available_min_dc_current_amp;

    // 0x24 — limit-active flags
    limits.is_capacitor_temp_limit_active = _feedback_data.general_io_msg.is_capacitor_temp_limit_active;
    limits.is_dc_current_limit_active = _feedback_data.general_io_msg.is_dc_current_limit_active;
    limits.is_drive_enable_limit_active = _feedback_data.general_io_msg.is_drive_enable_limit_active;
    limits.is_igbt_accel_temp_limit_active = _feedback_data.general_io_msg.is_igbt_accel_temp_limit_active;
    limits.is_igbt_temp_limit_active = _feedback_data.general_io_msg.is_igbt_temp_limit_active;
    limits.is_input_voltage_limit_active = _feedback_data.general_io_msg.is_input_voltage_limit_active;
    limits.is_motor_accel_temp_limit_active = _feedback_data.general_io_msg.is_motor_accel_temp_limit_active;
    limits.is_motor_temp_limit_active = _feedback_data.general_io_msg.is_motor_temp_limit_active;
    limits.is_rpm_min_limit_active = _feedback_data.general_io_msg.is_rpm_min_limit_active;
    limits.is_rpm_max_limit_active = _feedback_data.general_io_msg.is_rpm_max_limit_active;
    limits.is_power_limit_active = _feedback_data.general_io_msg.is_power_limit_active;

    return limits;
}

/* ---------- DTI-specific diagnostics ---------- */

DTIFaultCode_e InverterInterface::getFaultCode() const
{
    return _feedback_data.temp_and_fault_msg.fault_code;
}

const StatusGeneralIOMsg_s& InverterInterface::getIOStatus() const
{
    return _feedback_data.general_io_msg;
}

const StatusGeneralControlMsg_s& InverterInterface::getControlStatus() const
{
    return _feedback_data.general_control_msg;
}

const InverterStatusMessages_s& InverterInterface::getAllInverterData() const
{
    return _feedback_data;
}


/* ---------- Private helpers ---------- */

void InverterInterface::_markReceived(unsigned long curr_millis)
{
    _last_recv_millis = curr_millis;
    _has_received = true;
}

float InverterInterface::_convertTorqueToCurrent(float motor_torque_nm) const
{
    if (!std::isfinite(motor_torque_nm)) // false if infinity, -infinity, or NaN
    {
        return 0.0f;
    }

    // Id should only be zero in the base speed region or negative in the field weakening region
    float id_actual = std::min(_feedback_data.foc_current_msg.id_apk, 0.0f);

    float lambda_effective = _dti_params.lambda_pm_wb +
                            (_dti_params.ld_henries - _dti_params.lq_henries) * // negative value, -128 uH
                            id_actual; // normally negative

    float iqff = (2.0f * motor_torque_nm) / (3.0f * static_cast<float>(_dti_params.num_pole_pairs) * lambda_effective);

    return std::hypot(id_actual, iqff);
}

float InverterInterface::_convertCurrentToTorque(float id_apk, float iq_apk) const
{
    return 1.5f * static_cast<float>(_dti_params.num_pole_pairs) *
        (_dti_params.lambda_pm_wb * iq_apk + (_dti_params.ld_henries - _dti_params.lq_henries) * id_apk * iq_apk);
}

float InverterInterface::_convertRPMToERPM(float motor_speed_rpm) const
{
    return motor_speed_rpm * static_cast<float>(_dti_params.num_pole_pairs);
}

uint16_t InverterInterface::_packCANID(uint8_t packet_id) const
{
    return static_cast<uint16_t>((static_cast<uint16_t>(packet_id) << 5) | (_node_id & 0x1F));
}