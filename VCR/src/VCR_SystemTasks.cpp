#include "VCR_SystemTasks.hpp"


void initializeAllSystems()
{
    /**
     * @note etl::delegate stores a pointer to the lambda it is created from, not a copy. The lambdas below are static so
     *       they outlive this function; a non-static lambda would leave every delegate dangling once we return.
     */

    /* ---------- Drivetrain System ---------- */
    static auto isHVStatusOK = []() -> bool
    {
        unsigned long curr_millis = sys_time::hal_millis();
        ACUCANInterfaceData_s acu_data = ACUInterfaceInstance::instance().getLatestData(curr_millis);

        if (!acu_data.is_bms_voltages_heartbeat_ok)
        {
            return false;
        }

        float pack_voltage = acu_data.pack_voltage;

        for (InverterInterface* inverter : {&fl_inverter_interface, &fr_inverter_interface, &rl_inverter_interface, &rr_inverter_interface})
        {
            InverterStatus_s status = inverter->getStatus(curr_millis);

            // is_hv_present also requires the inverter to be connected, so a stale 0 V reading can't match a 0 V pack
            if (!status.is_hv_present || (std::fabs(status.dc_bus_voltage - pack_voltage) > VCRSystems::MAX_PACK_TO_INVERTER_VOLTAGE_DIFF_V))
            {
                return false;
            }
        }

        return true;
    };

    DrivetrainInstance::create(makeInverterFuncts(), etl::delegate<bool()>::create(isHVStatusOK));

    /* ---------- Drivebrain Control System ---------- */
    // Must exist before the VSM below binds handleDrivetrainCommand to it
    VCRControlsInstance::create(&DrivetrainInstance::instance(), VCRSystems::MAX_ALLOWED_DB_LATENCY_MS);

    /* ---------- Vehicle State Machine ---------- */
    static auto setMotorsIdle = []() -> void
    {
        fl_inverter_interface.setMotorIdle();
        fr_inverter_interface.setMotorIdle();
        rl_inverter_interface.setMotorIdle();
        rr_inverter_interface.setMotorIdle();
    };

    static auto isVehicleLatched = []() -> bool
    {
        return vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.FL > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE
               && vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.FR > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE
               && vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.RL > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE
               && vcr_data.system_data.drivetrain_data.measured_hv_bus_voltage.RR > VCRSystems::INVERTER_MINIMUM_HV_VOLTAGE;
    };

    static auto startBuzzer = []() -> void
    {
        VCFInterfaceInstance::instance().enqueueDashboardStatesCANMessage(true, false, false);
    };
    static auto sendRecalibratePedalsMessage = []() -> void
    {
        VCFInterfaceInstance::instance().enqueueDashboardStatesCANMessage(false, true, false);
    };
    static auto sendRecalibrateSteeringMessage = []() -> void
    {
        VCFInterfaceInstance::instance().enqueueDashboardStatesCANMessage(false, false, true);
    };
    static auto isDrivetrainFaulted = []() -> bool
    {
        return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::FAULTED;
    };
    static auto isDrivetrainNotConnected = []() -> bool
    {
        return DrivetrainInstance::instance().getCurrentState() == DrivetrainState_e::NOT_CONNECTED;
    };

    VCFInterface& vcf = VCFInterfaceInstance::instance();

    VehicleStateMachineInstance::create(
        etl::delegate<void()>::create(setMotorsIdle),
        etl::delegate<bool()>::create(isVehicleLatched),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isStartButtonPressed>(vcf),
        etl::delegate<void()>::create(startBuzzer),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isBrakePressed>(vcf),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isPedalsHeartbeatNotOk>(vcf),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isSteeringHeartbeatNotOk>(vcf),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::resetPedalsHeartbeat>(vcf),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::resetSteeringHeartbeat>(vcf),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isPedalsRecalibratePressed>(vcf),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::isSteeringRecalibratePressed>(vcf),
        etl::delegate<void()>::create(sendRecalibratePedalsMessage),
        etl::delegate<void()>::create(sendRecalibrateSteeringMessage),
        etl::delegate<bool()>::create(isDrivetrainFaulted),
        etl::delegate<bool()>::create(isDrivetrainNotConnected),
        etl::delegate<void(bool, unsigned long)>::create<VCRControls, &VCRControls::handleDrivetrainCommand>(VCRControlsInstance::instance()),
        etl::delegate<bool()>::create<RSSInterface, &RSSInterface::isGoButtonPressed>(RSSInterfaceInstance::instance()));
}

HT_TASK::TaskResponse tickStateMachine(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VehicleStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}
