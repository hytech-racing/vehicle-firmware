#include "VCR_SystemTasks.h"

void initialize_all_systems()
{

    /* Delegate Function Definitions For Drivetrain System State Machine */
    etl::delegate<void(torque_nm)> setMotorsTorque =
        etl::delegate<void(torque_nm)>::create<InverterInterface, &InverterInterface::setMotorsTorque>(InverterInterfaceInstance::instance()
    );

    etl::delegate<void(speed_rpm)> setMotorsSpeed =
        etl::delegate<void(speed_rpm)>::create<InverterInterface, &InverterInterface::setMotorsSpeed>(InverterInterfaceInstance::instance()
    );

    etl::delegate<void(speed_rpm)> setMotorsIdle =
        etl::delegate<void()>::create<InverterInterface, &InverterInterface::setMotorsIdle>(InverterInterfaceInstance::instance()
    );

    etl::delegate<void(bool)> requestEnable =
        etl::delegate<void(bool)>::create<InverterInterface, &InverterInterface::requestEnable>(InverterInterfaceInstance::instance()
    );

    etl::delegate<bool(DrivetrainControlMode_e)> isReportedModeMatchingDT =
        etl::delegate<bool(DrivetrainControlMode_e)>::create<InverterInterface, &InverterInterface::isReportedModeMatchingDT>(InverterInterfaceInstance::instance()
    );

    etl::delegate<InverterStatus_s()> getInverterStatus =
        etl::delegate<InverterStatus_s()>::create<InverterInterface, &InverterInterface::getStatus>(InverterInterfaceInstance::instance()
    );

    etl::delegate<MotorMechanics_s()> getMotorMechanics =
        etl::delegate<MotorMechanics_s()>::create<InverterInterface, &InverterInterface::getMotorMechanics>(InverterInterfaceInstance::instance()
    );

    /// TODO:
    etl::delegate<bool()> is_hv_status_ok =
        etl::delegate<bool()>::create([]() -> bool { return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::NOT_CONNECTED; }
    );

    InverterInterfaceFuncts_s inverter_functs = {
        setMotorsTorque,
        setMotorsSpeed,
        setMotorsIdle,
        requestEnable,
        isReportedModeMatchingDT,
        getInverterStatus,
        getMotorMechanics
    };

    DrivetrainInstance::create(
        VCRSystems::CONTROL_MODE_MISTMATCH_THRESHOLD_MS,
        inverter_functs,
        is_hv_status_ok
    );


    /* Delegate Function Definitions For Vehicle State Machine */
    etl::delegate<void(bool)> set_ef_pin_active = etl::delegate<void(bool)>::create(
        [](bool set_active) { digitalWrite(VCRInterfaces::INVERTER_ENABLE_PIN, static_cast<int>(set_active)); });

    etl::delegate<bool()> drivetrain_error_present =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_error_present>(DrivetrainInstance::instance());

    etl::delegate<bool()> drivetrain_ready =
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_ready>(DrivetrainInstance::instance());

    etl::delegate<void(bool, bool)> handle_drivetrain_command =
        etl::delegate<void(bool, bool)>::create<VCRControls, &VCRControls::handle_drivetrain_command>(VCRControlsInstance::instance());

    etl::delegate<bool()> drivetrain_reset_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_drivetrain_reset_pressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> recalibrate_pedals_button_pressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_recalibrate_pedals_button_pressed>(VCFInterfaceInstance::instance());

    etl::delegate<void()> reset_dt_error =
        etl::delegate<void()>::create<DrivetrainSystem, &DrivetrainSystem::reset_dt_error>(DrivetrainInstance::instance());


    // Set motors enable taken from above

    etl::delegate<bool()> isVehicleLatched =
        etl::delegate<bool()>::create<ACUInterface, &ACUInterface::isVehicleLatched>(ACUInterfaceInstance::instance()
    );

    etl::delegate<bool()> isRTDPressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isStartButtonPressed>(VCFInterfaceInstance::instance());

    etl::delegate<void()> startBuzzer =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::sendStartBuzzerCANMessage>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> isBrakePressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isBrakePressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> isPedalsTimedOut =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isPedalsHeartbeatNotOk>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> isSteeringTimedOut =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isSteeringHeartbeatNotOk>(VCFInterfaceInstance::instance());

    etl::delegate<void()> resetPedalsHeartbeat =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::resetPedalsHeartbeat>(VCFInterfaceInstance::instance());

    etl::delegate<void()> resetSteeringHeartbeat =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::resetSteeringHeartbeat>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> isPedalsRecalibratePressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isPedalsRecalibratePressed>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> isSteeringRecalibratePressed =
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isSteeringRecalibratePressed>(VCFInterfaceInstance::instance());

    etl::delegate<void()> sendRecalibratePedalsMessage =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::enqueuePedalsRecalibrateCANMessage>(VCFInterfaceInstance::instance());

    etl::delegate<void()> sendRecalibrateSteeringMessage =
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::enqueueSteeringRecalibrateCANMessage>(VCFInterfaceInstance::instance());

    etl::delegate<bool()> isDrivetrainFaulted =
        etl::delegate<bool()>::create([]() -> bool { return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::FAULTED; });

    etl::delegate<bool()> isDrivetrainNotConnected =
        etl::delegate<bool()>::create([]() -> bool { return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::NOT_CONNECTED; });

    VehicleStateMachineInstance::create(
        setMotorsIdle,
        isVehicleLatched,
        isRTDPressed,
        startBuzzer,
        isBrakePressed,
        isPedalsTimedOut,
        isSteeringTimedOut,
        resetPedalsHeartbeat,
        resetSteeringHeartbeat,
        isPedalsRecalibratePressed,
        isSteeringRecalibratePressed,
        sendRecalibratePedalsMessage,
        sendRecalibrateSteeringMessage,
        isDrivetrainFaulted,
        isDrivetrainNotConnected
    );

    /* ---------- Drivebrain Control System ---------- */
    VCRControlsInstance::create(&DrivetrainInstance::instance(), VCRSystems::MAX_ALLOWED_DB_LATENCY_MS);

    /* ---------- Drivetrain System ---------- */
    DrivetrainInstance::create(inverter_functs, set_ef_pin_active);
}

HT_TASK::TaskResponse tickStateMachine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    VehicleStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}