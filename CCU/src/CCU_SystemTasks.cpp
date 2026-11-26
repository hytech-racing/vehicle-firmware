#include "CCU_SystemTasks.hpp"


void initializeAllSystems()
{
    Level2SystemInstance::create(
        Level2InterfaceInstance::instance(),
        ADCInterfaceInstance::instance(),
        WatchdogInterfaceInstance::instance(),
        Level2SystemThresholds_s{});

    MainChargeSystemInstance::create(
        CCUSystems::MAX_120V_CURRENT_AMP,
        CCUSystems::MAX_240V_CURRENT_AMP,
        CCUConstants::MAX_CELL_CUTOFF_TEMP_CELSIUS,
        CCUConstants::MAX_BOARD_CUTOFF_TEMP_CELSIUS,
        ChargeSystemCCCVParams_s{
            .target_pack_voltage = CCUSystems::TARGET_PACK_VOLTAGE,
            .cv_detect_band_voltage = CCUSystems::CV_DETECT_BAND_VOLTAGE,
            .termination_current_amp = CCUSystems::CHARGE_TERMINATION_CURRENT_AMP,
            .termination_time_ms = CCUSystems::CHARGE_TERMINATION_TIME_MS,
        });
    MainChargeSystemInstance::instance().init(sys_time::hal_millis());

    /* State Machine Initialization */
    /* Delegate Function Definitions */
    etl::delegate<bool()> is120ConditionsOK =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().check_120_conditions(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is120Switched =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().is_120_switched(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is240Switched =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().is_240_switched(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> isShutdownDHigh = etl::delegate<bool()>::create([]() -> bool { return (ADCInterfaceInstance::instance().isShutdownDHigh()); });

    etl::delegate<bool()> is240ConditionsOK =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().check_240_conditions(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> isStateB2Ready = etl::delegate<bool()>::create(
        []() -> bool
        { return Level2SystemInstance::instance().check_state_B2_conditions(ADCInterfaceInstance::instance(), Level2InterfaceInstance::instance()); });

    etl::delegate<bool()> isStateC2Ready = etl::delegate<bool()>::create(
        []() -> bool
        { return Level2SystemInstance::instance().check_state_C2_conditions(ADCInterfaceInstance::instance(), Level2InterfaceInstance::instance()); });

    etl::delegate<bool()> resetErrorRequested =
        etl::delegate<bool()>::create([]() -> bool { return ADCInterfaceInstance::instance().isResetErrorsButtonPressed(sys_time::hal_millis()); });

    etl::delegate<void()> setSWShutdownHigh = etl::delegate<void()>::create([]() -> void { WatchdogInterfaceInstance::instance().setSWShutdownPinHigh(); });

    etl::delegate<void()> setSWShutdownLow = etl::delegate<void()>::create([]() -> void { WatchdogInterfaceInstance::instance().setSWShutdownPinLow(); });

    etl::delegate<void()> setStartChargeHigh = etl::delegate<void()>::create([]() -> void { Level2InterfaceInstance::instance().setStartCharge(HIGH); });

    etl::delegate<void()> setStartChargeLow = etl::delegate<void()>::create([]() -> void { Level2InterfaceInstance::instance().setStartCharge(LOW); });

    etl::delegate<void()> resetStartupTimeMs =
        etl::delegate<void()>::create([]() -> void { MainChargeSystemInstance::instance().init(sys_time::hal_millis()); });

    ChargerStateMachineInstance::create(
        is120ConditionsOK,
        is120Switched,
        is240Switched,
        isShutdownDHigh,
        is240ConditionsOK,
        isStateB2Ready,
        isStateC2Ready,
        resetErrorRequested,
        setSWShutdownHigh,
        setSWShutdownLow,
        setStartChargeHigh,
        setStartChargeLow,
        resetStartupTimeMs,
        sys_time::hal_millis());
}


HT_TASK::TaskResponse tickStateMachineTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ChargerStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse calculateChargeCurrentTask(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    MainChargeSystemInstance::instance().calculateChargeCurrent(
        CCUConstants::MAX_PACK_VOLTAGE,
        CCUConstants::MAX_CELL_CUTOFF_VOLTAGE,
        RotaryEncoderInterfaceInstance::instance().getValue(),
        ChargerInterfaceInstance::instance().getOutputCurrentA(),
        ChargerInterfaceInstance::instance().isFaulted(),
        sys_time::hal_millis());

    // Report the charge enable decision to ACU (sent in CCU_STATUS)
    ACUInterfaceInstance::instance().setIsChargingEnabled(MainChargeSystemInstance::instance().isChargingEnabled());

    return HT_TASK::TaskResponse::YIELD;
}
