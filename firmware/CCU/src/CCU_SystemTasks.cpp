#include "CCU_SystemTasks.h"


void initialize_all_systems()
{
    Level2SystemInstance::create(Level2InterfaceInstance::instance(),
                                ADCInterfaceInstance::instance(),
                                WatchdogInterfaceInstance::instance(),
                                Level2SystemThresholds_s {}
    );

    MainChargeSystemInstance::create(CCUSystems::MAX_120V_CURRENT_AMP,
                                    CCUSystems::MAX_240V_CURRENT_AMP,
                                    CCUConstants::MAX_CELL_CUTOFF_TEMP_CELSIUS,
                                    CCUConstants::MAX_BOARD_CUTOFF_TEMP_CELSIUS
    );
    MainChargeSystemInstance::instance().init(sys_time::hal_millis());

    /* State Machine Initialization */
    /* Delegate Function Definitions */
    etl::delegate<bool()> is_120_conditions_ok = etl::delegate<bool()>::create([]() -> bool
                                                                                { return Level2SystemInstance::instance().check_120_conditions(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_120_switched = etl::delegate<bool()>::create([]() -> bool
                                                                                { return Level2SystemInstance::instance().is_120_switched(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_240_switched = etl::delegate<bool()>::create([]() -> bool
                                                                                { return Level2SystemInstance::instance().is_240_switched(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_shdn_D_high = etl::delegate<bool()>::create([]() -> bool
                                                                                { return (ADCInterfaceInstance::instance().read_shdn_D_voltage()); });

    etl::delegate<bool()> is_240_conditions_ok = etl::delegate<bool()>::create([]() -> bool
                                                                                { return Level2SystemInstance::instance().check_240_conditions(ADCInterfaceInstance::instance()); });

    etl::delegate<bool()> is_state_B2_ready = etl::delegate<bool()>::create([]() -> bool
                                                                                { return Level2SystemInstance::instance().check_state_B2_conditions(ADCInterfaceInstance::instance(), Level2InterfaceInstance::instance()); });

    etl::delegate<bool()> is_state_C2_ready = etl::delegate<bool()>::create([]() -> bool
                                                                                { return Level2SystemInstance::instance().check_state_C2_conditions(ADCInterfaceInstance::instance(), Level2InterfaceInstance::instance()); });

    etl::delegate<bool()> reset_error_requested = etl::delegate<bool()>::create([]() -> bool
                                                                                { return ADCInterfaceInstance::instance().is_reset_errors_button_pressed(sys_time::hal_millis()); });

    etl::delegate<void()> set_sw_shdn_high = etl::delegate<void()>::create([]() -> void
                                                                                { WatchdogInterfaceInstance::instance().set_sw_shdn_pin_high(); });

    etl::delegate<void()> set_sw_shdn_low = etl::delegate<void()>::create([]() -> void
                                                                                { WatchdogInterfaceInstance::instance().set_sw_shdn_pin_low(); });

    etl::delegate<void()> set_start_charge_high = etl::delegate<void()>::create([]() -> void
                                                                                { Level2InterfaceInstance::instance().set_start_charge(HIGH); });

    etl::delegate<void()> set_start_charge_low = etl::delegate<void()>::create([]() -> void
                                                                                { Level2InterfaceInstance::instance().set_start_charge(LOW); });

    etl::delegate<void()> reset_startup_time_ms = etl::delegate<void()>::create([]() -> void
                                                                                { MainChargeSystemInstance::instance().init(sys_time::hal_millis()); });

    ChargerStateMachineInstance::create(is_120_conditions_ok,
                                        is_120_switched,
                                        is_240_switched,
                                        is_shdn_D_high,
                                        is_240_conditions_ok,
                                        is_state_B2_ready,
                                        is_state_C2_ready,
                                        reset_error_requested,
                                        set_sw_shdn_high,
                                        set_sw_shdn_low,
                                        set_start_charge_high,
                                        set_start_charge_low,
                                        reset_startup_time_ms,
                                        sys_time::hal_millis()
    );
    
}


HT_TASK::TaskResponse tick_state_machine(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo)
{
    ChargerStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse calculate_charge_current(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    MainChargeSystemInstance::instance().calculate_charge_current(CCUConstants::MAX_PACK_VOLTAGE,
                                                                CCUConstants::MAX_CELL_CUTOFF_VOLTAGE,
                                                                RotaryEncoderInterfaceInstance::instance().get_value(),
                                                                sys_time::hal_millis()
    );

    return HT_TASK::TaskResponse::YIELD;
}