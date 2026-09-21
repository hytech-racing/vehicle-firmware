#include "VCR_SystemTasks.h"

void initialize_all_systems()
{

    veh_vec<InverterInterfaceFuncts_s> inverter_interfaces_functs = makeInverterFuncts();

    /* Delegate Function Definitions For Drivetrain System State Machine */
    etl::delegate<void(torque_nm)> setMotorTorque =
        etl::delegate<void(torque_nm)>::create<InverterInterface, &InverterInterface::setMotorTorque>(InverterInterfaceInstance::instance());

    etl::delegate<void(speed_rpm)> setMotorSpeed =
        etl::delegate<void(speed_rpm)>::create<InverterInterface, &InverterInterface::setMotorSpeed>(InverterInterfaceInstance::instance());

    etl::delegate<void(speed_rpm)> setMotorIdle =
        etl::delegate<void()>::create<InverterInterface, &InverterInterface::setMotorIdle>(InverterInterfaceInstance::instance());

    etl::delegate<void(bool)> requestEnable =
        etl::delegate<void(bool)>::create<InverterInterface, &InverterInterface::requestEnable>(InverterInterfaceInstance::instance());

    etl::delegate<bool(DrivetrainControlMode_e)> isReportedModeMatchingDT =
        etl::delegate<bool(DrivetrainControlMode_e)>::create<InverterInterface, &InverterInterface::isReportedModeMatchingDT>(InverterInterfaceInstance::instance());

    etl::delegate<InverterStatus_s()> getInverterStatus =
        etl::delegate<InverterStatus_s()>::create<InverterInterface, &InverterInterface::getStatus>(InverterInterfaceInstance::instance());

    etl::delegate<MotorMechanics_s()> getMotorMechanics =
        etl::delegate<MotorMechanics_s()>::create<InverterInterface, &InverterInterface::getMotorMechanics>(InverterInterfaceInstance::instance());

    /// TODO:
    etl::delegate<bool()> is_hv_status_ok =
        etl::delegate<bool()>::create([]() -> bool { return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::NOT_CONNECTED; });

    InverterInterfaceFuncts_s inverter_functs = {
        setMotorTorque,
        setMotorSpeed,
        setMotorIdle,
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
    etl::delegate<bool()> isVehicleLatched =
        etl::delegate<bool()>::create([]() -> bool { return vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.FL > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE &&
                                                            vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.FR > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE &&
                                                            vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.RL > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE &&
                                                            vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.RR > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE; });

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
    DrivetrainInstance::create(VCRSystems::CONTROL_MODE_MISTMATCH_THRESHOLD_MS,
                                        inverter_interfaces_functs,
                                        is_hv_status_ok()
    );
}

HT_TASK::TaskResponse tickStateMachine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    VehicleStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}