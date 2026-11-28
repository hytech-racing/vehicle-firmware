#include "InverterInterface.hpp"

#include "VCRCANInterfaceImpl.hpp"


/**
 * Getters for the data
 */
InverterStatus_s InverterInterface::getStatus() const
{
    InverterStatus_s status_struct = _feedback_data.status;
    _feedback_data.status.new_data = false;
    return status_struct;
}

InverterTemps_s InverterInterface::getTemps() const
{
    InverterTemps_s temps_struct = _feedback_data.temps;
    _feedback_data.temps.new_data = false;
    return temps_struct;
}

InverterPower_s InverterInterface::getPower() const
{
    InverterPower_s power_struct = _feedback_data.power;
    _feedback_data.power.new_data = false;
    return power_struct;
}

MotorMechanics_s InverterInterface::getMotorMechanics() const
{
    MotorMechanics_s mm_struct = _feedback_data.motor_mechanics;
    _feedback_data.motor_mechanics.new_data = false;
    return mm_struct;
}


InverterControlFeedback_s InverterInterface::getControlParams() const
{
    InverterControlFeedback_s cf_struct = _feedback_data.control_feedback;
    _feedback_data.control_feedback.new_data = false;
    return cf_struct;
}

InverterFeedbackData_s InverterInterface::getAllInverterData() const
{
    return _feedback_data;
}

/**
 * receiving CAN messages
 */
void InverterInterface::receiveInvStatus(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    // Unpack the message
    INV1_STATUS_t unpacked_msg;
    Unpack_INV1_STATUS_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueueMsg(&unpacked_msg, &Pack_INV1_STATUS_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id);

    // Update inverter interface with new data
    _feedback_data.status.connected = true; // Will set to true once first CAN message has been received
    _feedback_data.status.system_ready = unpacked_msg.system_ready;
    _feedback_data.status.error = unpacked_msg.error;
    _feedback_data.status.warning = unpacked_msg.warning;
    _feedback_data.status.quit_dc_on = unpacked_msg.quit_dc_on;
    _feedback_data.status.dc_on = unpacked_msg.dc_on;
    _feedback_data.status.quit_inverter_on = unpacked_msg.quit_inverter_on;
    _feedback_data.status.derating_on = unpacked_msg.derating_on;
    _feedback_data.status.dc_bus_voltage = unpacked_msg.dc_bus_voltage;
    _feedback_data.status.diagnostic_number = unpacked_msg.diagnostic_number;
    _feedback_data.status.hv_present = _feedback_data.status.dc_bus_voltage > _inverter_params.minimum_hv_voltage;

    _feedback_data.status.new_data = true;
    _feedback_data.status.last_recv_millis = curr_millis;
}

void InverterInterface::receiveInvTemps(const CAN_message_t& can_msg, unsigned long curr_millis)
{

    // Unpack the message
    INV1_TEMPS_t unpacked_msg;
    Unpack_INV1_TEMPS_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueueMsg(&unpacked_msg, &Pack_INV1_TEMPS_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id);

    // Update inverter interface with new data
    _feedback_data.temps.igbt_temp = HT_CAN_igbt_temp_ro_fromS(unpacked_msg.igbt_temp_ro);
    _feedback_data.temps.inverter_temp = HT_CAN_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
    _feedback_data.temps.motor_temp = HT_CAN_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);

    _feedback_data.temps.new_data = true;
    _feedback_data.temps.last_recv_millis = curr_millis;
}

void InverterInterface::receiveInvDynamics(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    // Unpack the message
    INV1_DYNAMICS_t unpacked_msg;
    Unpack_INV1_DYNAMICS_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueueMsg(&unpacked_msg, &Pack_INV1_DYNAMICS_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id);

    // Update inverter interface with new data
    _feedback_data.motor_mechanics.actual_power = unpacked_msg.actual_power_w; // NOLINT (watts)
    _feedback_data.motor_mechanics.actual_torque = HT_CAN_actual_torque_nm_ro_fromS(unpacked_msg.actual_torque_nm_ro);
    _feedback_data.motor_mechanics.actual_speed = unpacked_msg.actual_speed_rpm;
    _feedback_data.motor_mechanics.new_data = true;
    _feedback_data.motor_mechanics.last_recv_millis = curr_millis;
}

void InverterInterface::receiveInvPower(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    // Unpack the message
    INV1_POWER_t unpacked_msg;
    Unpack_INV1_POWER_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueueMsg(&unpacked_msg, &Pack_INV1_POWER_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id);

    // Update inverter interface with new data
    _feedback_data.power.active_power = unpacked_msg.active_power_w;       // NOLINT (watts)
    _feedback_data.power.reactive_power = unpacked_msg.reactive_power_var; // NOLINT (watts)

    _feedback_data.power.new_data = true;
    _feedback_data.power.last_recv_millis = curr_millis;
}

void InverterInterface::receiveInvFeedback(const CAN_message_t& can_msg, unsigned long curr_millis)
{
    // Unpack the message
    INV1_FEEDBACK_t unpacked_msg;
    Unpack_INV1_FEEDBACK_ht_can(&unpacked_msg, can_msg.buf, can_msg.len);
    CAN_util::enqueueMsg(&unpacked_msg, &Pack_INV1_FEEDBACK_ht_can, VCRCANInterfaceInstance::instance().telem_can_tx_buffer, can_msg.id);

    // Update inverter interface with new data
    _feedback_data.control_feedback.speed_control_kp = unpacked_msg.speed_control_kp;
    _feedback_data.control_feedback.speed_control_ki = unpacked_msg.speed_control_ki;
    _feedback_data.control_feedback.speed_control_kd = unpacked_msg.speed_control_kd;

    _feedback_data.control_feedback.new_data = true;
    _feedback_data.control_feedback.last_recv_millis = curr_millis;
}

/**
 * Sending CAN messages
 */
void InverterInterface::sendInvSetpointCommand()
{
    INV1_CONTROL_INPUT_t msg_out;

    msg_out.speed_setpoint_rpm = _inverter_control_inputs.speed_rpm_setpoint;
    msg_out.positive_torque_limit_ro = HT_CAN_positive_torque_limit_ro_toS(_inverter_control_inputs.positive_torque_limit);
    msg_out.negative_torque_limit_ro = HT_CAN_negative_torque_limit_ro_toS(_inverter_control_inputs.negative_torque_limit);

    CAN_util::enqueueMsg(
        &msg_out,
        &Pack_INV1_CONTROL_INPUT_ht_can,
        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
        _inverter_ids.inv_control_input_id);
    CAN_util::enqueueMsg(
        &msg_out,
        &Pack_INV1_CONTROL_INPUT_ht_can,
        VCRCANInterfaceInstance::instance().telem_can_tx_buffer,
        _inverter_ids.inv_control_input_id);
}

void InverterInterface::sendInvControlWord()
{
    INV1_CONTROL_WORD_t msg_out;

    msg_out.driver_enable = _inverter_control_word.driver_enable;
    msg_out.hv_enable = _inverter_control_word.hv_enable;
    msg_out.inverter_enable = _inverter_control_word.inverter_enable;
    msg_out.remove_error = _inverter_control_word.remove_error;

    CAN_util::enqueueMsg(
        &msg_out,
        &Pack_INV1_CONTROL_WORD_ht_can,
        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
        _inverter_ids.inv_control_word_id);
}

void InverterInterface::sendInvControlParams()
{
    INV1_CONTROL_PARAMETER_t msg_out;

    msg_out.speed_control_kd = _inverter_control_params.speed_control_kp;
    msg_out.speed_control_ki = _inverter_control_params.speed_control_ki;
    msg_out.speed_control_kd = _inverter_control_params.speed_control_kd;

    CAN_util::enqueueMsg(
        &msg_out,
        &Pack_INV1_CONTROL_PARAMETER_ht_can,
        VCRCANInterfaceInstance::instance().inverter_can_tx_buffer,
        _inverter_ids.inv_control_parameter_id);
}

/**
 * Methods for use as inverter functs
 */
void InverterInterface::setSpeed(float desired_rpm, float torque_limit_nm)
{
    _inverter_control_inputs.speed_rpm_setpoint = static_cast<int16_t>(desired_rpm);
    _inverter_control_inputs.positive_torque_limit = ::fabs(torque_limit_nm);
    _inverter_control_inputs.negative_torque_limit = -1.0f * ::fabs(torque_limit_nm);
}

void InverterInterface::setIdle()
{
    _inverter_control_inputs.negative_torque_limit = 0;
    _inverter_control_inputs.positive_torque_limit = 0;
    _inverter_control_inputs.speed_rpm_setpoint = 0;
}

void InverterInterface::setInverterControlWord(InverterControlWord_s control_word)
{
    _inverter_control_word.driver_enable = control_word.driver_enable;
    _inverter_control_word.hv_enable = control_word.hv_enable;
    _inverter_control_word.inverter_enable = control_word.inverter_enable;
    _inverter_control_word.remove_error = control_word.remove_error;
}
