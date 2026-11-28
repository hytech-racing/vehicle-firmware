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
        CCUConstants::MAX_BOARD_CUTOFF_TEMP_CELSIUS);
    MainChargeSystemInstance::instance().init(sys_time::hal_millis());

    /* State Machine Initialization */
    /* Delegate Function Definitions */
    etl::delegate<bool()> is_120_conditions_ok =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().check120Conditions(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_120_switched =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().is120Switched(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_240_switched =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().is240Switched(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_shutdown_d_high = etl::delegate<bool()>::create([]() -> bool { return (ADCInterfaceInstance::instance().isShutdownDHigh()); });

    etl::delegate<bool()> is_240_conditions_ok =
        etl::delegate<bool()>::create([]() -> bool { return Level2SystemInstance::instance().check240Conditions(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_state_b2_ready = etl::delegate<bool()>::create(
        []() -> bool
        { return Level2SystemInstance::instance().checkStateB2Conditions(ADCInterfaceInstance::instance(), Level2InterfaceInstance::instance()); });

    etl::delegate<bool()> is_state_c2_ready = etl::delegate<bool()>::create(
        []() -> bool
        { return Level2SystemInstance::instance().checkStateC2Conditions(ADCInterfaceInstance::instance(), Level2InterfaceInstance::instance()); });

    etl::delegate<bool()> reset_error_requested =
        etl::delegate<bool()>::create([]() -> bool { return ADCInterfaceInstance::instance().isResetErrorsButtonPressed(sys_time::hal_millis()); });

    etl::delegate<void()> set_sw_shutdown_high = etl::delegate<void()>::create([]() -> void { WatchdogInterfaceInstance::instance().setSWShutdownPinHigh(); });

    etl::delegate<void()> set_sw_shutdown_low = etl::delegate<void()>::create([]() -> void { WatchdogInterfaceInstance::instance().setSWShutdownPinLow(); });

    etl::delegate<void()> set_start_charge_high = etl::delegate<void()>::create([]() -> void { Level2InterfaceInstance::instance().setStartCharge(HIGH); });

    etl::delegate<void()> set_start_charge_low = etl::delegate<void()>::create([]() -> void { Level2InterfaceInstance::instance().setStartCharge(LOW); });

    etl::delegate<void()> reset_startup_time_ms =
        etl::delegate<void()>::create([]() -> void { MainChargeSystemInstance::instance().init(sys_time::hal_millis()); });

    ChargerStateMachineInstance::create(
        is_120_conditions_ok,
        is_120_switched,
        is_240_switched,
        is_shutdown_d_high,
        is_240_conditions_ok,
        is_state_b2_ready,
        is_state_c2_ready,
        reset_error_requested,
        set_sw_shutdown_high,
        set_sw_shutdown_low,
        set_start_charge_high,
        set_start_charge_low,
        reset_startup_time_ms,
        sys_time::hal_millis());
}


HT_TASK::TaskResponse tickStateMachineTask(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    ChargerStateMachineInstance::instance().tickStateMachine(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse calculateChargeCurrentTask(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
{
    MainChargeSystemInstance::instance().calculateChargeCurrent(
        CCUConstants::MAX_PACK_VOLTAGE,
        CCUConstants::MAX_CELL_CUTOFF_VOLTAGE,
        RotaryEncoderInterfaceInstance::instance().getValue(),
        sys_time::hal_millis());

    return HT_TASK::TaskResponse::YIELD;
}
