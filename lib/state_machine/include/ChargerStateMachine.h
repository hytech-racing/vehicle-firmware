#ifndef CHARGERSTATEMACHINE_H
#define CHARGERSTATEMACHINE_H

/* ETL Library */
#include "etl/singleton.h"
#include <etl/delegate.h>

/* Local Interface Includes */
#include "ACUInterface.h"
#include "ChargerInterface.h"

enum class ChargerState_e
{
    STARTUP = 0,            // Default state: LV turned on, not charging. Expected Values: CP = 0 ; PP = 5 ; 240_En = HIGH ; 240_OK = LOW
    CHECK_SWITCH,           // Check where the switch is using JMP_Read. Startup values should be present
    CHARGE_120_UNLATCHED,   // All 120V charging conditions are ready, just waiting for user to engage charging by setting CCU_OK high
    CHARGING_120,           // 120V charging with no balancing
    CHECK_240_B2_OK,        // Correlates to EVSE State B2 (intial pwm from charger). If okay, set START_CHARGE high
    CHECK_240_C2_OK,        // Correlates to EVSE State C/C2.
    CHARGE_240_UNLATCHED,   // All 240V charging conditions are ready, just waiting for user to engage charging by setting CCU_OK high
    CHARGING_240,           // 240 V charging with no balancing
    ERROR,
    NUM_CHARGER_STATES
};

class ChargerStateMachine
{
public:

    ChargerStateMachine(
        etl::delegate<bool()> is_120_conditions_ok,
        etl::delegate<bool()> is_120_switched,
        etl::delegate<bool()> is_240_switched,
        etl::delegate<bool()> is_shdn_D_high,
        etl::delegate<bool()> is_240_conditions_ok,
        etl::delegate<bool()> is_state_B2_ready,
        etl::delegate<bool()> is_state_C2_ready,
        etl::delegate<bool()> reset_error_requested,
        etl::delegate<void()> set_sw_shdn_high,
        etl::delegate<void()> set_sw_shdn_low,
        etl::delegate<void()> set_start_charge_high,
        etl::delegate<void()> set_start_charge_low,
        etl::delegate<void()> reset_startup_time_ms,
        uint32_t current_millis
    ) :
        _current_state(ChargerState_e::STARTUP),
        _last_state_changed_time(current_millis),
        _is_120_conditions_ok(is_120_conditions_ok),
        _is_120_switched(is_120_switched),
        _is_240_switched(is_240_switched),
        _is_shdn_D_high(is_shdn_D_high),
        _is_240_conditions_ok(is_240_conditions_ok),
        _is_state_B2_ready(is_state_B2_ready),
        _is_state_C2_ready(is_state_C2_ready),
        _reset_error_requested(reset_error_requested),
        _set_sw_shdn_high(set_sw_shdn_high),
        _set_sw_shdn_low(set_sw_shdn_low),
        _set_start_charge_high(set_start_charge_high),
        _set_start_charge_low(set_start_charge_low),
        _reset_startup_time_ms(reset_startup_time_ms)
    {};

    ChargerState_e tick_state_machine(unsigned long current_millis);

    /**
     * @return current CCU state
    */
    ChargerState_e get_state() { return _current_state; }

    /**
     * @return string for printing out the state
    */
    const char* get_state_name();

private:

    const unsigned long state_transition_delay_ms = 2500UL; // ms

    void _set_state(ChargerState_e new_state, unsigned long current_millis);

    /**
     * The function run upon the entry of the charger into a new state.
     * @param new_state The state in which we are entering.
     */
    void _handle_entry_logic(ChargerState_e new_state, unsigned long current_millis);

    /**
     * The function run upon the exit of a state.
     * @param prev_state the state in which we are leaving.
     */
    void _handle_exit_logic(ChargerState_e prev_state, unsigned long current_millis);

    ChargerState_e _current_state;
    unsigned long _last_state_changed_time; // time of last state change

    // Lamdas for state machine abstraction, functions defined in main
    etl::delegate<bool()> _is_120_conditions_ok;
    etl::delegate<bool()> _is_120_switched;
    etl::delegate<bool()> _is_240_switched;
    etl::delegate<bool()> _is_shdn_D_high;
    etl::delegate<bool()> _is_240_conditions_ok;
    etl::delegate<bool()> _is_state_B2_ready;
    etl::delegate<bool()> _is_state_C2_ready;
    etl::delegate<bool()> _reset_error_requested;

    /// @brief setters
    etl::delegate<void()> _set_sw_shdn_high;
    etl::delegate<void()> _set_sw_shdn_low;
    etl::delegate<void()> _set_start_charge_high;
    etl::delegate<void()> _set_start_charge_low;
    etl::delegate<void()> _reset_startup_time_ms;

};

using ChargerStateMachineInstance = etl::singleton<ChargerStateMachine>;

#endif