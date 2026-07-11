#include "ChargerStateMachine.h"

// logic for changing states - still need to account for dial_position
ChargerState_e ChargerStateMachine::tick_state_machine(unsigned long current_millis)
{
    switch (_current_state) // takes in the _current_state variables and matches it to each case
    {
        case ChargerState_e::STARTUP:
        {
            if (current_millis - _last_state_changed_time < state_transition_delay_ms)
            {
                break;
            }
            if (!_is_120_conditions_ok())
            {
                _set_state(ChargerState_e::ERROR, current_millis);
                break;
            }
            else
            {
                _set_state(ChargerState_e::CHECK_SWITCH, current_millis);
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

            if (!_is_120_conditions_ok())
            {
                _set_state(ChargerState_e::ERROR, current_millis);
                break;
            }

            if (_is_120_switched())
            {
                _set_state(ChargerState_e::CHARGE_120_UNLATCHED, current_millis);
                break;
            }

            if (_is_240_switched())
            {
                _set_state(ChargerState_e::CHECK_240_B2_OK, current_millis);
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

            if (!_is_120_conditions_ok() || !_is_120_switched())
            {
                _set_state(ChargerState_e::ERROR, current_millis);
                break;
            }

            if (_is_shdn_D_high())
            {
                _set_state(ChargerState_e::CHARGING_120, current_millis);
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

            if (!_is_120_conditions_ok() || !_is_120_switched())
            {
                _set_state(ChargerState_e::ERROR, current_millis);
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

            if (_is_state_B2_ready())
            {
                _set_state(ChargerState_e::CHECK_240_C2_OK, current_millis);
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

            if (!_is_240_conditions_ok())
            {
                _set_state(ChargerState_e::ERROR, current_millis);
                break;
            }


            if (_is_state_C2_ready())
            {
                _set_state(ChargerState_e::CHARGE_240_UNLATCHED, current_millis);
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

            if (!_is_state_C2_ready() || !_is_240_conditions_ok())
            {
                _set_state(ChargerState_e::ERROR, current_millis);
                break;
            }

            if (_is_shdn_D_high())
            {
                _set_state(ChargerState_e::CHARGING_240, current_millis);
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

            if (!_is_state_C2_ready() || !_is_240_conditions_ok())
            {
                _set_state(ChargerState_e::ERROR, current_millis);
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

            if (_reset_error_requested())
            {
                _set_state(ChargerState_e::STARTUP, current_millis);
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

void ChargerStateMachine::_set_state(ChargerState_e new_state, unsigned long current_millis)
{
    _handle_exit_logic(_current_state, current_millis);
    _current_state = new_state;
    _handle_entry_logic(_current_state, current_millis);

    // update any time there is a state change
    _last_state_changed_time = current_millis;
}

void ChargerStateMachine::_handle_exit_logic(ChargerState_e prev_state, unsigned long current_millis)
{
    switch(prev_state)
    {
        case ChargerState_e::CHECK_240_B2_OK:
        {
            _set_start_charge_high();
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
void ChargerStateMachine::_handle_entry_logic(ChargerState_e new_state, unsigned long current_millis)
{
    switch(new_state)
    {
        case ChargerState_e::STARTUP:
        {
            break;
        }
        case ChargerState_e::CHARGE_120_UNLATCHED:
        {
            _set_sw_shdn_high();
            break;
        }
        case ChargerState_e::CHARGE_240_UNLATCHED:
        {
            _set_sw_shdn_high();
            break;
        }
        case ChargerState_e::CHARGING_240:
        {
            _reset_startup_time_ms();
            break;
        }
        case ChargerState_e::ERROR:
        {
            _set_sw_shdn_low();
            _set_start_charge_low();
            break;
        }
        case ChargerState_e::CHECK_SWITCH: break;
        case ChargerState_e::CHARGING_120: break;
        case ChargerState_e::CHECK_240_B2_OK: break;
        case ChargerState_e::CHECK_240_C2_OK: break;
        default: break;
    }
}

const char* ChargerStateMachine::get_state_name()
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