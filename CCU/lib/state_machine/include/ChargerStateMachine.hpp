#ifndef CHARGER_STATE_MACHINE
#define CHARGER_STATE_MACHINE

/* ETL Library */
#include <etl/singleton.h>
#include <etl/delegate.h>

/* Local Interface Includes */
#include "ACUInterface.hpp"
#include "ChargerInterface.hpp"

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
        etl::delegate<bool()> is120ConditionsOK,
        etl::delegate<bool()> is120Switched,
        etl::delegate<bool()> is240Switched,
        etl::delegate<bool()> isShutdownDHigh,
        etl::delegate<bool()> is240ConditionsOK,
        etl::delegate<bool()> isStateB2Ready,
        etl::delegate<bool()> isStateC2Ready,
        etl::delegate<bool()> resetErrorRequested,
        etl::delegate<void()> setSWShutdownHigh,
        etl::delegate<void()> setSWShutdownLow,
        etl::delegate<void()> setStartChargeHigh,
        etl::delegate<void()> setStartChargeLow,
        etl::delegate<void()> resetStartupTimeMs,
        uint32_t current_millis
    ) :
        _current_state(ChargerState_e::STARTUP),
        _last_state_changed_time(current_millis),
        _is120ConditionsOK(is120ConditionsOK),
        _is120Switched(is120Switched),
        _is240Switched(is240Switched),
        _isShutdownDHigh(isShutdownDHigh),
        _is240ConditionsOK(is240ConditionsOK),
        _isStateB2Ready(isStateB2Ready),
        _isStateC2Ready(isStateC2Ready),
        _resetErrorRequested(resetErrorRequested),
        _setSWShutdownHigh(setSWShutdownHigh),
        _setSWShutdownLow(setSWShutdownLow),
        _setStartChargeHigh(setStartChargeHigh),
        _setStartChargeLow(setStartChargeLow),
        _resetStartupTimeMs(resetStartupTimeMs)
    {};

    ChargerState_e tickStateMachine(unsigned long current_millis);

    /**
     * @return current CCU state
    */
    ChargerState_e getState() { return _current_state; }

    /**
     * @return string for printing out the state
    */
    const char* getStateName();

private:

    const unsigned long state_transition_delay_ms = 2500UL; // ms

    void _setState(ChargerState_e new_state, unsigned long current_millis);

    /**
     * The function run upon the entry of the charger into a new state.
     * @param new_state The state in which we are entering.
     */
    void _handleEntryLogic(ChargerState_e new_state, unsigned long current_millis);

    /**
     * The function run upon the exit of a state.
     * @param prev_state the state in which we are leaving.
     */
    void _handleExitLogic(ChargerState_e prev_state, unsigned long current_millis);

    ChargerState_e _current_state;
    unsigned long _last_state_changed_time; // time of last state change

    // Lamdas for state machine abstraction, functions defined in main
    etl::delegate<bool()> _is120ConditionsOK;
    etl::delegate<bool()> _is120Switched;
    etl::delegate<bool()> _is240Switched;
    etl::delegate<bool()> _isShutdownDHigh;
    etl::delegate<bool()> _is240ConditionsOK;
    etl::delegate<bool()> _isStateB2Ready;
    etl::delegate<bool()> _isStateC2Ready;
    etl::delegate<bool()> _resetErrorRequested;

    /// @brief setters
    etl::delegate<void()> _setSWShutdownHigh;
    etl::delegate<void()> _setSWShutdownLow;
    etl::delegate<void()> _setStartChargeHigh;
    etl::delegate<void()> _setStartChargeLow;
    etl::delegate<void()> _resetStartupTimeMs;

};

using ChargerStateMachineInstance = etl::singleton<ChargerStateMachine>;

#endif