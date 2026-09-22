#include "VCR_SystemTasks.h"

void initialize_all_systems()
{

    /* Delegate Function Definitions For Drivetrain System State Machine */
    /// TODO:
    etl::delegate<bool()> is_hv_status_ok =
        etl::delegate<bool()>::create([]() -> bool { return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::NOT_CONNECTED; });

    veh_vec<InverterInterfaceFuncts_s> inverter_functs = makeInverterFuncts();

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
        etl::delegate<void()>::create([]() -> void { VCFInterfaceInstance::instance().enqueueDashboardStatesCANMessage(true, false, false); });

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
        etl::delegate<void()>::create([]() -> void { VCFInterfaceInstance::instance().enqueueDashboardStatesCANMessage(false, true, false); });

    etl::delegate<void()> sendRecalibrateSteeringMessage =
        etl::delegate<void()>::create([]() -> void { VCFInterfaceInstance::instance().enqueueDashboardStatesCANMessage(false, false, true); });

    etl::delegate<bool()> isDrivetrainFaulted =
        etl::delegate<bool()>::create([]() -> bool { return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::FAULTED; });

    etl::delegate<bool()> isDrivetrainNotConnected =
        etl::delegate<bool()>::create([]() -> bool { return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::NOT_CONNECTED; });

    etl::delegate<void(bool, unsigned long)> handleDrivetrainCommand =
        etl::delegate<void(bool, unsigned long)>::create<VCRControls, &VCRControls::handleDrivetrainCommand>(VCRControlsInstance::instance());

    VehicleStateMachineInstance::create(
        setMotorIdle,
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
        isDrivetrainNotConnected,
        handleDrivetrainCommand
    );

    /* ---------- Drivebrain Control System ---------- */
    VCRControlsInstance::create(&DrivetrainInstance::instance(), VCRSystems::MAX_ALLOWED_DB_LATENCY_MS);

}

HT_TASK::TaskResponse tickStateMachine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    VehicleStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}