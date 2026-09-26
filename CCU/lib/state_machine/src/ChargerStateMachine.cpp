#include "ChargerStateMachine.hpp"


// logic for changing states - still need to account for dial_position
ChargerState_e ChargerStateMachine::tickStateMachine(unsigned long current_millis)
{
    switch (_current_state) // takes in the _current_state variables and matches it to each case
    {
        case ChargerState_e::STARTUP:
        {
            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break;
            }
            if (!_is120ConditionsOK())
            {
                _setState(ChargerState_e::ERROR, current_millis);
                break;
            }
            else
            {
                _setState(ChargerState_e::CHECK_SWITCH, current_millis);
                break;
            }
        }
        case ChargerState_e::CHECK_SWITCH:
        {
            /**
             * Purpose of this state is to check the switch position
             *
             * Error Cases:
             * 1) Startup/120V conditions become errored (CP no longer zero, PP no longer 5, etc.)
             *
             * NOTE:
             * 240_OK low + JMP_Read high = 120V
             * 240_OK low = JMP_Read low  = 240V
             */

            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break; // delay to control the state transitions, cannot state transition too fast
            }

            if (!_is120ConditionsOK())
            {
                _setState(ChargerState_e::ERROR, current_millis);
                break;
            }

            if (_is120Switched())
            {
                _setState(ChargerState_e::CHARGE_120_UNLATCHED, current_millis);
                break;
            }

            if (_is240Switched())
            {
                _setState(ChargerState_e::CHECK_240_B2_OK, current_millis);
                break;
            }

            break;
        }
        case ChargerState_e::CHARGE_120_UNLATCHED:
        {
            /**
             * Purpose of this state is to allow user to have more control about when we start charging. Use the latch button to engage charging.
             * However, in this state we are ready for 120V Charging.
             *
             * Error Cases:
             * 1) Startup values error (CP no longer zero, PP no longer 5, etc.)
             * 2) Someone switches to 240V charging
             */

            // // Check w david and adish, but this delay is technicaly bad because we want to immediatly detect errors
            // // and you cannot state transition without physically hitting latch
            // if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            // {
            //     break;
            // }

            if (!_is120ConditionsOK() || !_is120Switched())
            {
                _setState(ChargerState_e::ERROR, current_millis);
                break;
            }

            if (_isShutdownDHigh())
            {
                _setState(ChargerState_e::CHARGING_120, current_millis);
                break;
            }

            break;
        }
        case ChargerState_e::CHARGING_120:
        {
            /**
             * In this state we are performing 120V charging.
             *
             * Error Cases:
             * 1) Startup values error (CP no longer zero, PP no longer 5, etc.)
             * 2) Someone switches to 240V charging w/o delatching
             */

            if (!_is120ConditionsOK() || !_is120Switched())
            {
                _setState(ChargerState_e::ERROR, current_millis);
                break;
            }

            break;
        }
        case ChargerState_e::CHECK_240_B2_OK:
        {
            /**
             * This state checks for EVSE State B2. If okay, set START_CHARGE high (done in exit logic).
             *
             * NOTE: In state B2, 240_Ok = HIGH.
             *
             * Error Cases:
             * 1) Someone switches to 120V charging, ie. 240_Ok and JP_OUT_READ goes LOW
             */

            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break;
            }

            if (_isStateB2Ready())
            {
                _setState(ChargerState_e::CHECK_240_C2_OK, current_millis);
                break;
            }

            break;
        }
        case ChargerState_e::CHECK_240_C2_OK:
        {
            /**
             * This state checks for EVSE State C/C2.
             *
             * Error Cases:
             * 1) Someone switches to 120V charging, ie. 240_Ok and JP_OUT_READ goes LOW
             */

            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break;
            }

            if (!_is240ConditionsOK())
            {
                _setState(ChargerState_e::ERROR, current_millis);
                break;
            }


            if (_isStateC2Ready())
            {
                _setState(ChargerState_e::CHARGE_240_UNLATCHED, current_millis);
                break;
            }

            break;
        }
        case ChargerState_e::CHARGE_240_UNLATCHED:
        {
            /**
             * Purpose of this state is to allow user to have more control about when we start charging. Use the latch button to engage charging.
             * However, in this state we are ready for 240V Charging.
             *
             * Error Cases:
             * 1) State C2 values error (incorrect pwm values, etc.)
             * 2) Someone switches to 120V charging, ie. 240_Ok and JP_OUT_READ goes LOW
             */

            // Check w david and adish, but this delay is technicaly bad because we want to immediatly detect errors
            // and you cannot state transition without physically hitting latch
            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break;
            }

            if (!_isStateC2Ready() || !_is240ConditionsOK())
            {
                _setState(ChargerState_e::ERROR, current_millis);
                break;
            }

            if (_isShutdownDHigh())
            {
                _setState(ChargerState_e::CHARGING_240, current_millis);
                break;
            }

            break;
        }
        case ChargerState_e::CHARGING_240:
        {
            /**
             * In this state we are performing 240V charging.
             *
             * Error Cases:
             * 1) State C values error (CP no longer zero, PP no longer 5, etc.)
             * 2) Someone switches to 120V charging w/o delatching
             */
            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break;
            }

            if (!_isStateC2Ready() || !_is240ConditionsOK())
            {
                _setState(ChargerState_e::ERROR, current_millis);
                break;
            }

            break;
        }
        case ChargerState_e::ERROR:
        {
            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break;
            }

            if (_resetErrorRequested())
            {
                _setState(ChargerState_e::STARTUP, current_millis);
                break;
            }

            break;
        }
        default: // Should never occur
        {
            break;
        }

    }
    return _current_state;
}

void ChargerStateMachine::_setState(ChargerState_e new_state, unsigned long current_millis)
{
    _handleExitLogic(_current_state, current_millis);
    _current_state = new_state;
    _handleEntryLogic(_current_state, current_millis);

    // update any time there is a state change
    _last_state_changed_time = current_millis;
}

void ChargerStateMachine::_handleExitLogic(ChargerState_e prev_state, unsigned long current_millis)
{
    switch(prev_state)
    {
        case ChargerState_e::CHECK_240_B2_OK:
        {
            _setStartChargeHigh();
            break;
        }
        case ChargerState_e::STARTUP: break;
        case ChargerState_e::CHECK_SWITCH: break;
        case ChargerState_e::CHARGE_120_UNLATCHED: break;
        case ChargerState_e::CHARGING_120: break; // only exit would be to error, probably implement as enter logic
        case ChargerState_e::CHECK_240_C2_OK: break;
        case ChargerState_e::CHARGE_240_UNLATCHED: break;
        case ChargerState_e::CHARGING_240: break; // only exit would be to error, probably implement as enter logic
        case ChargerState_e::ERROR: break;
        default: break;
    }
}

//make sure each state is reset before you enter it
void ChargerStateMachine::_handleEntryLogic(ChargerState_e new_state, unsigned long current_millis)
{
    switch(new_state)
    {
        case ChargerState_e::STARTUP:
        {
            break;
        }
        case ChargerState_e::CHARGE_120_UNLATCHED:
        {
            _setSWShutdownHigh();
            break;
        }
        case ChargerState_e::CHARGE_240_UNLATCHED:
        {
            _setSWShutdownHigh();
            break;
        }
        case ChargerState_e::CHARGING_240:
        {
            _resetStartupTimeMs();
            break;
        }
        case ChargerState_e::ERROR:
        {
            _setSWShutdownLow();
            _setStartChargeLow();
            break;
        }
        case ChargerState_e::CHECK_SWITCH: break;
        case ChargerState_e::CHARGING_120: break;
        case ChargerState_e::CHECK_240_B2_OK: break;
        case ChargerState_e::CHECK_240_C2_OK: break;
        default: break;
    }
}

const char* ChargerStateMachine::getStateName()
{
    switch (_current_state)
    {
        case ChargerState_e::STARTUP:
        {
            return "STARTUP";
        }
        case ChargerState_e::CHARGING_120:
        {
            return "CHARGING 120";
        }
        case ChargerState_e::CHARGING_240:
        {
            return "CHARGING 240";
        }
        case ChargerState_e::ERROR:
        {
            return "ERROR";
        }
        case ChargerState_e::CHECK_SWITCH:
        {
            return "CHECK SWITCH";
        }
        case ChargerState_e::CHARGE_120_UNLATCHED:
        {
            return "CHARGE 120 UNLATCHED";
        }
        case ChargerState_e::CHECK_240_B2_OK:
        {
            return "CHECK 240 B2 OK";
        }
        case ChargerState_e::CHECK_240_C2_OK:
        {
            return "CHECK 240 C2 OK";
        }
        case ChargerState_e::CHARGE_240_UNLATCHED:
        {
            return "CHARGE 240 UNLATCHED";
        }
        default:
            return "UNKNOWN";
    }
}