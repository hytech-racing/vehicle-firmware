#include "VCFInterface.h"
#include "VCRCANInterfaceImpl.h"

void VCFInterface::receive_pedals_message(const CAN_message_t &msg, unsigned long curr_millis)
{
    PEDALS_SYSTEM_DATA_t pedals_msg;
    Unpack_PEDALS_SYSTEM_DATA_hytech(&pedals_msg, &msg.buf[0], msg.len);
    _curr_data.stamped_pedals.pedals_data.implausibility_has_exceeded_max_duration =
        pedals_msg.implaus_exceeded_max_duration;

    _curr_data.stamped_pedals.pedals_data.brake_and_accel_pressed_implausibility_high =
        pedals_msg.brake_accel_implausibility;

    _curr_data.stamped_pedals.pedals_data.accel_is_implausible = pedals_msg.accel_implausible;
    _curr_data.stamped_pedals.pedals_data.brake_is_implausible = pedals_msg.brake_implausible;

    _curr_data.stamped_pedals.pedals_data.mech_brake_is_active = pedals_msg.mechanical_brake_active;
    _curr_data.stamped_pedals.pedals_data.brake_is_pressed = pedals_msg.brake_pedal_active;
    _curr_data.stamped_pedals.pedals_data.accel_is_pressed = pedals_msg.accel_pedal_active;

    _curr_data.stamped_pedals.pedals_data.accel_percent =
        HYTECH_accel_pedal_ro_fromS(static_cast<float>(pedals_msg.accel_pedal_ro));
    _curr_data.stamped_pedals.pedals_data.brake_percent =
        HYTECH_brake_pedal_ro_fromS(static_cast<float>(pedals_msg.brake_pedal_ro));
    _curr_data.stamped_pedals.last_recv_millis = curr_millis;

    // As long as we're using millis() function, loop overrun not a concern

    if (_curr_data.stamped_pedals.last_recv_millis == 0)
    {
        _first_received_message_heartbeat_init = true;
    }

    _curr_data.stamped_pedals.last_recv_millis = curr_millis;
}

void VCFInterface::receive_steering_message(const CAN_message_t &msg, unsigned long curr_millis)
{
    STEERING_DATA_t steering_msg;

    Unpack_STEERING_DATA_hytech(&steering_msg, &msg.buf[0], msg.len);
    _curr_data.stamped_steering.steering_data.analog_oor_implausibility = steering_msg.steering_analog_oor;
    _curr_data.stamped_steering.steering_data.both_sensors_fail = steering_msg.steering_both_sensors_fail;
    _curr_data.stamped_steering.steering_data.digital_oor_implausibility = steering_msg.steering_digital_oor;
    _curr_data.stamped_steering.steering_data.dtheta_exceeded_analog = steering_msg.steering_dtheta_exceeded_analog;
    _curr_data.stamped_steering.steering_data.dtheta_exceeded_digital = steering_msg.steering_dtheta_exceeded_digital;
    _curr_data.stamped_steering.steering_data.interface_sensor_error = steering_msg.steering_interface_sensor_error;
    _curr_data.stamped_steering.steering_data.output_steering_angle = HYTECH_steering_output_steering_angle_ro_fromS(steering_msg.steering_output_steering_angle_ro);
    _curr_data.stamped_steering.steering_data.analog_raw = steering_msg.steering_analog_raw;
    _curr_data.stamped_steering.steering_data.digital_raw = steering_msg.steering_digital_raw;

    if (_curr_data.stamped_steering.last_recv_millis == 0)
    {
        _first_received_message_heartbeat_init = true; // TODO: can this be the same as pedals?
    }

    _curr_data.stamped_steering.last_recv_millis = curr_millis;
}

void VCFInterface::receive_dashboard_message(const CAN_message_t &msg, unsigned long curr_millis)
{
    DASH_INPUT_t dash_msg;
    Unpack_DASH_INPUT_hytech(&dash_msg, &msg.buf[0], msg.len);
    _curr_data.dash_input_state.btn_dim_read_is_pressed = dash_msg.dim_button;
    _curr_data.dash_input_state.preset_btn_is_pressed = dash_msg.preset_button; // pedal recalibration button
    _curr_data.dash_input_state.mc_reset_btn_is_pressed = dash_msg.motor_controller_cycle_button;
    _curr_data.dash_input_state.start_btn_is_pressed = dash_msg.start_button;
    _curr_data.dash_input_state.data_btn_is_pressed = dash_msg.data_button_is_pressed;
    // _curr_data.dash_input_state.left_paddle_is_pressed = dash_msg.left_shifter_button;
    // _curr_data.dash_input_state.right_paddle_is_pressed = dash_msg.right_shifter_button;
    // _curr_data.dash_input_state.mode_btn_is_pressed = dash_msg.mode_button; // change torque limit
    _curr_data.dash_input_state.dial_state = static_cast<ControllerMode_e>(dash_msg.dash_dial_mode);
}

void VCFInterface::receive_front_suspension_message(const CAN_message_t &msg, unsigned long curr_millis)
{
    FRONT_SUSPENSION_t front_suspension_msg;
    Unpack_FRONT_SUSPENSION_hytech(&front_suspension_msg, &msg.buf[0], msg.len);
    _curr_data.front_loadcell_data.FL_loadcell_analog = front_suspension_msg.fl_load_cell;
    _curr_data.front_suspot_data.FL_sus_pot_analog = HYTECH_fl_shock_pot_ro_fromS(front_suspension_msg.fl_shock_pot_ro);
    _curr_data.front_loadcell_data.FR_loadcell_analog = front_suspension_msg.fr_load_cell;
    _curr_data.front_suspot_data.FR_sus_pot_analog = HYTECH_fr_shock_pot_ro_fromS(front_suspension_msg.fr_shock_pot_ro);

    _curr_data.front_loadcell_data.valid_FL_sample = true; // only sent over CAN if valid from VCF
    _curr_data.front_loadcell_data.valid_FR_sample = true; // or send validities over CAN
}

void VCFInterface::reset_pedals_heartbeat()
{
    _curr_data.stamped_pedals.heartbeat_ok = true;
}

void VCFInterface::reset_steering_heartbeat()
{
    _curr_data.stamped_steering.heartbeat_ok = true;
}

VCFCANInterfaceData_s VCFInterface::get_latest_data() const
{

    // only in the situation where the hearbeat has yet to be established or the heartbeat is ok do we re-evaluate the heartbeat.
    // if hearbeat is is not ok, the only thing that should be able to reset it is the state machine via the reset_pedals_heartbeat function
    if (_first_received_message_heartbeat_init || _curr_data.stamped_pedals.heartbeat_ok || _curr_data.stamped_steering.heartbeat_ok)
    {
        _first_received_message_heartbeat_init = false;
        _curr_data.stamped_pedals.heartbeat_ok = ((sys_time::hal_millis() - _curr_data.stamped_pedals.last_recv_millis) < _max_heartbeat_interval_ms);
        _curr_data.stamped_steering.heartbeat_ok = ((sys_time::hal_millis() - _curr_data.stamped_steering.last_recv_millis) < _max_heartbeat_interval_ms);
    }
    else
    {
        _curr_data.stamped_pedals.heartbeat_ok = false;
        _curr_data.stamped_steering.heartbeat_ok = false;
    }

    return _curr_data;
}

void VCFInterface::send_buzzer_start_message()
{
    DASHBOARD_BUZZER_CONTROL_t ctrl = {};
    ctrl.dash_buzzer_flag = true;
    ctrl.in_pedal_calibration_state = false;
    ctrl.in_steering_calibration_state = false;
    ctrl.torque_limit_enum_value = 0xFF; // MAX_VALUE indicates "ignore this value" //NOLINT
    CAN_util::enqueue_msg(&ctrl,
                        &Pack_DASHBOARD_BUZZER_CONTROL_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}

void VCFInterface::send_recalibrate_pedals_message()
{
    DASHBOARD_BUZZER_CONTROL_t ctrl = {};
    ctrl.dash_buzzer_flag = false;
    ctrl.in_pedal_calibration_state = true;
    ctrl.in_steering_calibration_state = false;
    ctrl.torque_limit_enum_value = 0xFF; // MAX_VALUE indicates "ignore this value" //NOLINT
    CAN_util::enqueue_msg(&ctrl,
                        &Pack_DASHBOARD_BUZZER_CONTROL_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}

void VCFInterface::send_recalibrate_steering_message()
{
    DASHBOARD_BUZZER_CONTROL_t ctrl = {};
    ctrl.dash_buzzer_flag = false;
    ctrl.in_pedal_calibration_state = false;
    ctrl.in_steering_calibration_state = true;
    ctrl.torque_limit_enum_value = 0xFF; // MAX_VALUE indicates "ignore this value" //NOLINT
    CAN_util::enqueue_msg(&ctrl,
                        &Pack_DASHBOARD_BUZZER_CONTROL_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}

void VCFInterface::enqueue_torque_mode_LED_message(TorqueLimit_e torque_limit)
{
    DASHBOARD_BUZZER_CONTROL_t ctrl = {};
    ctrl.dash_buzzer_flag = false;
    ctrl.in_pedal_calibration_state = false;
    ctrl.in_steering_calibration_state = false;
    ctrl.torque_limit_enum_value = (uint8_t) torque_limit;
    CAN_util::enqueue_msg(&ctrl,
                        &Pack_DASHBOARD_BUZZER_CONTROL_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}

void VCFInterface::enqueue_vehicle_state_message(VehicleState_e vehicle_state, DrivetrainState_e drivetrain_state, bool db_is_in_ctrl)
{
    CAR_STATES_t state = {};
    state.vehicle_state = static_cast<uint8_t>(vehicle_state);
    state.drivetrain_state = static_cast<uint8_t>(drivetrain_state);
    state.drivebrain_in_control = db_is_in_ctrl;
    CAN_util::enqueue_msg(&state,
                        &Pack_CAR_STATES_hytech,
                        VCRCANInterfaceInstance::instance().telem_can_tx_buffer
    );
}