#include "VCR_SystemTasks.hpp"


DrivetrainSystem::InverterFuncts_s fl_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { fl_inverter_interface.setSpeed(rpm, torque_nm); },
    .set_idle = []() { fl_inverter_interface.setIdle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { fl_inverter_interface.setInverterControlWord(cw); },
    .get_status = []() { return fl_inverter_interface.getStatus(); },
    .get_motor_mechanics =
        []()
    {
        return fl_inverter_interface.getMotorMechanics();
    }};

DrivetrainSystem::InverterFuncts_s fr_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { fr_inverter_interface.setSpeed(rpm, torque_nm); },
    .set_idle = []() { fr_inverter_interface.setIdle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { fr_inverter_interface.setInverterControlWord(cw); },
    .get_status = []() { return fr_inverter_interface.getStatus(); },
    .get_motor_mechanics =
        []()
    {
        return fr_inverter_interface.getMotorMechanics();
    }};

DrivetrainSystem::InverterFuncts_s rl_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { rl_inverter_interface.setSpeed(rpm, torque_nm); },
    .set_idle = []() { rl_inverter_interface.setIdle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { rl_inverter_interface.setInverterControlWord(cw); },
    .get_status = []() { return rl_inverter_interface.getStatus(); },
    .get_motor_mechanics =
        []()
    {
        return rl_inverter_interface.getMotorMechanics();
    }};

DrivetrainSystem::InverterFuncts_s rr_inverter_functs = {
    .set_speed = [](float rpm, float torque_nm) { rr_inverter_interface.setSpeed(rpm, torque_nm); },
    .set_idle = []() { rr_inverter_interface.setIdle(); },
    .set_inverter_control_word = [](InverterControlWord_s cw) { rr_inverter_interface.setInverterControlWord(cw); },
    .get_status = []() { return rr_inverter_interface.getStatus(); },
    .get_motor_mechanics =
        []()
    {
        return rr_inverter_interface.getMotorMechanics();
    }};

veh_vec<DrivetrainSystem::InverterFuncts_s> inverter_functs(fl_inverter_functs, fr_inverter_functs, rl_inverter_functs, rr_inverter_functs);

etl::delegate<void(bool)> set_ef_pin_active =
    etl::delegate<void(bool)>::create([](bool set_active) { digitalWrite(VCRInterfaces::INVERTER_ENABLE_PIN, static_cast<int>(set_active)); });


void initializeAllSystems()
{
    /* ---------- Drivetrain System ---------- */
    DrivetrainInstance::create(inverter_functs, set_ef_pin_active);

    /* ---------- Drivebrain Control System ---------- */
    VCRControlsInstance::create(&DrivetrainInstance::instance(), VCRSystems::MAX_ALLOWED_DB_LATENCY_MS);

    /* Delegate Function Definitions */
    etl::delegate<bool()> hv_over_threshold =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::hvOverThreshold>(DrivetrainInstance::instance());

    etl::delegate<bool()> start_button_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isStartButtonPressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> brake_pressed = etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isBrakePressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> drivetrain_error_present =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrainErrorPresent>(DrivetrainInstance::instance());

    etl::delegate<bool()> drivetrain_ready =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrainReady>(DrivetrainInstance::instance());

    etl::delegate<void()> send_buzzer_start_message =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::sendBuzzerStartMsg>(VCFInterfaceInstance::instance());

    etl::delegate<void()> send_recalibrate_pedals_message =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::sendRecalibratePedalsMsg>(VCFInterfaceInstance::instance());

    etl::delegate<void(bool, bool)> handle_drivetrain_command =
        etl::delegate<void(bool, bool)>::create<VCRControls, &VCRControls::handleDrivetrainCommand>(VCRControlsInstance::instance());

    etl::delegate<bool()> pedals_heartbeat_not_ok =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isPedalsHeartbeatNotOk>(VCFInterfaceInstance::instance());

    etl::delegate<void()> reset_pedals_heartbeat =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::resetPedalsHeartbeat>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> drivetrain_reset_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isDrivetrainResetPressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> recalibrate_pedals_button_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isRecalibratePedalsButtonPressed>(VCFInterfaceInstance::instance());

    etl::delegate<void()> reset_dt_error = etl::delegate<void()>::create<DrivetrainSystem, &DrivetrainSystem::resetDtError>(DrivetrainInstance::instance());

    etl::delegate<void()> send_recalibrate_steering_message =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::sendRecalibrateSteeringMsg>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> recalibrate_steering_button_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isRecalibrateSteeringButtonPressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> steering_heartbeat_not_ok =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isSteeringHeartbeatNotOk>(VCFInterfaceInstance::instance());

    etl::delegate<void()> reset_steering_heartbeat =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::resetSteeringHeartbeat>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> is_go_button_pressed =
        etl::delegate<bool()>::create<RSSInterface, &RSSInterface::isGoButtonPressed>(RSSInterfaceInstance::instance());

    VehicleStateMachineInstance::create(
        hv_over_threshold,
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
        reset_steering_heartbeat,
        is_go_button_pressed);
}

/**
 * TODO: Understand asyn better, but not sure ticking state machine needs to/should go there.
 */
// HT_TASK::TaskResponse tickStateMachine(const unsigned long &sys_micros, const HT_TASK::TaskInfo &task_info)
// {
//     VehicleStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());

//     return HT_TASK::TaskResponse::YIELD;
// }
