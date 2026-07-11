#include "VCR_SystemTasks.h"


DrivetrainSystem::InverterFuncts_s fl_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { fl_inverter_interface.set_speed(rpm, torque_nm); },
    .set_idle = []() { fl_inverter_interface.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { fl_inverter_interface.set_inverter_control_word(cw); },
    .get_status = []() { return fl_inverter_interface.get_status(); },
    .get_motor_mechanics = []() { return fl_inverter_interface.get_motor_mechanics(); }
};

DrivetrainSystem::InverterFuncts_s fr_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { fr_inverter_interface.set_speed(rpm, torque_nm); },
    .set_idle = []() { fr_inverter_interface.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { fr_inverter_interface.set_inverter_control_word(cw); },
    .get_status = []() { return fr_inverter_interface.get_status(); },
    .get_motor_mechanics = []() { return fr_inverter_interface.get_motor_mechanics(); }
};

DrivetrainSystem::InverterFuncts_s rl_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { rl_inverter_interface.set_speed(rpm, torque_nm); },
    .set_idle = []() { rl_inverter_interface.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { rl_inverter_interface.set_inverter_control_word(cw); },
    .get_status = []() { return rl_inverter_interface.get_status(); },
    .get_motor_mechanics = []() { return rl_inverter_interface.get_motor_mechanics(); }
};

DrivetrainSystem::InverterFuncts_s rr_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { rr_inverter_interface.set_speed(rpm, torque_nm); },
    .set_idle = []() { rr_inverter_interface.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { rr_inverter_interface.set_inverter_control_word(cw); },
    .get_status = []() { return rr_inverter_interface.get_status(); },
    .get_motor_mechanics = []() { return rr_inverter_interface.get_motor_mechanics(); }
};

veh_vec<DrivetrainSystem::InverterFuncts_s> inverter_functs(fl_inverter_functs, fr_inverter_functs, rl_inverter_functs, rr_inverter_functs);

etl::delegate<void(bool)> set_ef_pin_active = etl::delegate<void(bool)>::create(
    [](bool set_active) { digitalWrite(VCRInterfaces::INVERTER_ENABLE_PIN, static_cast<int>(set_active)); });


void initialize_all_systems()
{
    /* Delegate Function Definitions */
    etl::delegate<bool()> hv_over_threshold =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::hv_over_threshold>(DrivetrainInstance::instance());

    etl::delegate<bool()> start_button_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_start_button_pressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> brake_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_brake_pressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> drivetrain_error_present =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_error_present>(DrivetrainInstance::instance());

    etl::delegate<bool()> drivetrain_ready =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_ready>(DrivetrainInstance::instance());

    etl::delegate<void()> send_buzzer_start_message =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_buzzer_start_message>(VCFInterfaceInstance::instance());

    etl::delegate<void()> send_recalibrate_pedals_message =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_recalibrate_pedals_message>(VCFInterfaceInstance::instance());

    etl::delegate<void(bool, bool)> handle_drivetrain_command =
        etl::delegate<void(bool, bool)>::create<VCRControls, &VCRControls::handle_drivetrain_command>(VCRControlsInstance::instance());

    etl::delegate<bool()> pedals_heartbeat_not_ok =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_pedals_heartbeat_not_ok>(VCFInterfaceInstance::instance());

    etl::delegate<void()> reset_pedals_heartbeat =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::reset_pedals_heartbeat>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> drivetrain_reset_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_drivetrain_reset_pressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> recalibrate_pedals_button_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_recalibrate_pedals_button_pressed>(VCFInterfaceInstance::instance());

    etl::delegate<void()> reset_dt_error =
        etl::delegate<void()>::create<DrivetrainSystem, &DrivetrainSystem::reset_dt_error>(DrivetrainInstance::instance());

    etl::delegate<void()> send_recalibrate_steering_message =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_recalibrate_steering_message>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> recalibrate_steering_button_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_recalibrate_steering_button_pressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> steering_heartbeat_not_ok =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_steering_heartbeat_not_ok>(VCFInterfaceInstance::instance());

    etl::delegate<void()> reset_steering_heartbeat =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::reset_steering_heartbeat>(VCFInterfaceInstance::instance());

    VehicleStateMachineInstance::create(hv_over_threshold,
                                    start_button_pressed,
                                    brake_pressed,
                                    drivetrain_error_present,
                                    drivetrain_ready,
                                    send_buzzer_start_message,
                                    send_recalibrate_pedals_message,
                                    handle_drivetrain_command,
                                    pedals_heartbeat_not_ok,
                                    reset_pedals_heartbeat,
                                    drivetrain_reset_pressed,
                                    recalibrate_pedals_button_pressed,
                                    reset_dt_error,
                                    send_recalibrate_steering_message,
                                    recalibrate_steering_button_pressed,
                                    steering_heartbeat_not_ok,
                                    reset_steering_heartbeat
    );

    /* ---------- Drivebrain Control System ---------- */
    VCRControlsInstance::create(&DrivetrainInstance::instance(), VCRSystems::MAX_ALLOWED_DB_LATENCY_MS);

    /* ---------- Drivetrain System ---------- */
    DrivetrainInstance::create(inverter_functs, set_ef_pin_active);
}

/**
 * TODO: Understand asyn better, but not sure ticking state machine needs to/should go there.
 */
// HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
// {
//     VehicleStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());

//     return HT_TASK::TaskResponse::YIELD;
// }