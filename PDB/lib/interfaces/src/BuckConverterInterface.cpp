#include "BuckConverterInterface.hpp"

// =============================================================================
// DC/DC converters: control via EN (if connected), monitoring via PG (if the part has one).
//
// State machine (with PG):
//   DISABLED   --enable()-->          STARTING
//   STARTING   --PG goes high-->      ACTIVE
//   STARTING   --timeout, PG low-->   FAULT (STARTUP_TIMEOUT)
//   ACTIVE     --PG goes low-->       FAULT (POWER_LOST)
//   FAULT      --PG goes high-->      ACTIVE (counted as a recovery)
//   any        --restart()-->         RESTARTING --off time elapsed--> STARTING
//   any        --disable()-->         DISABLED
//
// Without PG, STARTING --startup time elapsed--> ACTIVE, and FAULT is never entered.
// Without EN, the rail starts in STARTING at init() and never reaches DISABLED / RESTARTING.
// =============================================================================


BuckConverterInterface::BuckConverterInterface(const std::array<BuckConverterDefinition_s, buck_converter_interface_default_params::NUM_BUCKS> &definitions) : _all_bucks{}
{
    bool is_filled[buck_converter_interface_default_params::NUM_BUCKS] = {};

    for (const BuckConverterDefinition_s &definition : definitions)
    {
        // Place by id so the list order doesn't matter; a bad or repeated id invalidates the whole config
        if (!_isValidIndex(definition.id) || is_filled[definition.id])
        {
            _is_config_valid = false;
            continue;
        }
        is_filled[definition.id] = true;

        BuckConverter_s &buck = _all_bucks[definition.id];
        buck.en_pin = definition.en;
        buck.pg_pin = definition.pg;
        buck.timing_params = definition.timing_params;
        buck.status.part = definition.part;
        buck.status.has_en_pin = (definition.en != buck_converter_interface_default_params::NOT_CONNECTED);
        buck.status.has_pg_pin = (definition.pg != buck_converter_interface_default_params::NOT_CONNECTED);
        buck.status.current_state = BuckConverterState_e::DISABLED;
        buck.status.current_fault = BuckConverterFaultType_e::NONE;
    }
}

HAL_StatusTypeDef BuckConverterInterface::init()
{
    if (!_is_config_valid)
    {
        return HAL_ERROR;
    }

    const time_ms curr_millis = HAL_GetTick();
    for (BuckConverter_s &buck : _all_bucks)
    {
        // PG / RESET: open-drain from the part, pulled up on the board
        if (buck.status.has_pg_pin)
        {
            pinMode(buck.pg_pin, INPUT);
        }

        if (buck.status.has_en_pin)
        {
            // Write the level LOW *before* making the pin an output so EN never glitches high
            _setEnablePin(buck, false);                  // Controlled rails stay off until enable()
            pinMode(buck.en_pin, OUTPUT);
            buck.status.is_enabled = false;
            buck.status.current_state = BuckConverterState_e::DISABLED;
        }
        else
        {
            buck.status.is_enabled = true;               // Always-on rail: already starting (or up) since power-on
            _beginStartup(buck, curr_millis);
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef BuckConverterInterface::enable(uint8_t index)
{
    if (!_isValidIndex(index) || !_all_bucks[index].status.has_en_pin)
    {
        return HAL_ERROR;
    }
    BuckConverter_s &buck = _all_bucks[index];

    if (buck.status.is_enabled || buck.status.current_state == BuckConverterState_e::RESTARTING)
    {
        return HAL_OK;  // Already on, or restart() is still discharging the output (updateBuckConverter() re-enables when it's done)
    }

    _turnOn(buck, HAL_GetTick());
    return HAL_OK;
}

HAL_StatusTypeDef BuckConverterInterface::disable(uint8_t index)
{
    if (!_isValidIndex(index) || !_all_bucks[index].status.has_en_pin)
    {
        return HAL_ERROR;
    }
    BuckConverter_s &buck = _all_bucks[index];

    _setEnablePin(buck, false);                 // EN low: switching stops and the output discharges (part-dependent)
    buck.status.is_enabled = false;
    buck.is_pg_raw_high = false;                // All supported parts hold PG low whenever EN is low
    buck.status.is_pg_high = false;
    buck.status.current_fault = BuckConverterFaultType_e::NONE;
    buck.status.current_state = BuckConverterState_e::DISABLED;
    return HAL_OK;
}

HAL_StatusTypeDef BuckConverterInterface::restart(uint8_t index)
{
    if (disable(index) != HAL_OK)               // Validates index / EN pin; sets DISABLED, overwritten to RESTARTING below
    {
        return HAL_ERROR;
    }
    BuckConverter_s &buck = _all_bucks[index];
    buck.restart_began_at_ms = HAL_GetTick();   // Off timer starts now; updateBuckConverter() re-enables after timing_params.restart_wait_duration_ms
    buck.status.current_state = BuckConverterState_e::RESTARTING;
    return HAL_OK;
}

void BuckConverterInterface::updateAllBuckConverters()
{
    for (uint8_t i = 0; i < buck_converter_interface_default_params::NUM_BUCKS; i++)
    {
        updateBuckConverter(i);
    }
}

void BuckConverterInterface::updateBuckConverter(uint8_t index)
{
    if (!_isValidIndex(index))
    {
        return;
    }
    BuckConverter_s &buck = _all_bucks[index];
    BuckConverterStatus_s &status = buck.status;
    const time_ms curr_millis = HAL_GetTick();

    // RESTARTING: wait out the discharge time, then re-enable.
    // Handled before the enabled check because EN is low during this state.
    if (status.current_state == BuckConverterState_e::RESTARTING)
    {
        if ((curr_millis - buck.restart_began_at_ms) >= buck.timing_params.restart_wait_duration_ms)
        {
            _turnOn(buck, curr_millis);  // RESTARTING -> STARTING; bypasses enable()'s restart guard
        }
        return;
    }

    // DISABLED: nothing to monitor, PG is held low by the part.
    if (!status.is_enabled)
    {
        return;
    }

    // No PG pin: the only thing we can know is that soft start should be over.
    if (!status.has_pg_pin)
    {
        if (status.current_state == BuckConverterState_e::STARTING && (curr_millis - buck.enabled_at_ms) >= buck.timing_params.startup_timeout_duration_ms)
        {
            status.current_state = BuckConverterState_e::ACTIVE;
        }
        return;
    }

    _updatePowerGood(buck, curr_millis);  // Refresh the debounced PG level

    switch (status.current_state)
    {
        case BuckConverterState_e::STARTING:
        {
            if (status.is_pg_high)
            {
                // PG rose and held
                status.current_state = BuckConverterState_e::ACTIVE;
            }
            else if (!buck.is_pg_raw_high && (curr_millis - buck.enabled_at_ms) >= buck.timing_params.startup_timeout_duration_ms)
            {
                // Timed out with PG still low. The !is_pg_raw_high check skips the timeout
                // if PG just rose and is still being debounced.
                _enterFault(buck, BuckConverterFaultType_e::STARTUP_TIMEOUT);
            }
            break;
        }

        case BuckConverterState_e::ACTIVE:
        {
            if (!status.is_pg_high)
            {
                // PG fell out of regulation and stayed low.
                // Causes: overload/short (hiccup), thermal shutdown, UVLO, OV (LM61495). PG can't tell which.
                _enterFault(buck, BuckConverterFaultType_e::POWER_LOST);
            }
            break;
        }

        case BuckConverterState_e::FAULT:
        {
            if (status.is_pg_high)
            {
                // Recovered without intervention: thermal cooled down, hiccup short cleared,
                // input came back above UVLO, or a very slow start finally finished.
                status.current_fault = BuckConverterFaultType_e::NONE;
                status.current_state = BuckConverterState_e::ACTIVE;
                status.recovery_count++;
            }
            break;
        }

        default:
        {
            break;
        }
    }
}

const BuckConverterStatus_s *BuckConverterInterface::getStatus(uint8_t index) const
{
    return _isValidIndex(index) ? &_all_bucks[index].status : nullptr;
}

BuckConverterState_e BuckConverterInterface::getState(uint8_t index) const
{
    return _isValidIndex(index) ? _all_bucks[index].status.current_state : BuckConverterState_e::DISABLED;
}

bool BuckConverterInterface::isPowerGood(uint8_t index) const
{
    return getState(index) == BuckConverterState_e::ACTIVE;
}

bool BuckConverterInterface::isPowerGoodPinHigh(uint8_t index) const
{
    return _isValidIndex(index) && _isPowerGoodPinSet(_all_bucks[index]);
}

bool BuckConverterInterface::allEnabledPowerGood() const
{
    for (const BuckConverter_s &buck : _all_bucks)
    {
        if (buck.status.is_enabled && buck.status.current_state != BuckConverterState_e::ACTIVE)
        {
            return false;
        }
    }
    return true;
}

bool BuckConverterInterface::anyFault() const
{
    for (const BuckConverter_s &buck : _all_bucks)
    {
        if (buck.status.current_state == BuckConverterState_e::FAULT)
        {
            return true;
        }
    }
    return false;
}

bool BuckConverterInterface::_isValidIndex(uint8_t index) const
{
    return index < buck_converter_interface_default_params::NUM_BUCKS;
}

void BuckConverterInterface::_setEnablePin(const BuckConverter_s &buck, bool level)
{
    if (buck.en_pin == buck_converter_interface_default_params::NOT_CONNECTED)
    {
        return;
    }
    digitalWrite(buck.en_pin, level ? HIGH : LOW);
}

bool BuckConverterInterface::_isPowerGoodPinSet(const BuckConverter_s &buck)
{
    if (buck.pg_pin == buck_converter_interface_default_params::NOT_CONNECTED)
    {
        return false;
    }
    return digitalRead(buck.pg_pin) == HIGH;
}

void BuckConverterInterface::_turnOn(BuckConverter_s &buck, time_ms curr_millis)
{
    _setEnablePin(buck, true);  // EN high: converter begins soft-start
    buck.status.is_enabled = true;
    _beginStartup(buck, curr_millis);
}

void BuckConverterInterface::_beginStartup(BuckConverter_s &buck, time_ms curr_millis)
{
    buck.enabled_at_ms = curr_millis;
    // Record the pin's current level so updateBuckConverter() can detect when it changes
    // Should be low for the bucks the MCU enables, but high for the ones that turn on when the board is powered by Hotswap
    buck.is_pg_raw_high = _isPowerGoodPinSet(buck);
    buck.pg_raw_changed_at_ms = curr_millis;         // Start/Restart the debounce timer from this moment
    buck.status.is_pg_high = false;                  // Nothing counts as "good" until updateBuckConverter() has seen PG hold high for the full debounce time
    buck.status.current_fault = BuckConverterFaultType_e::NONE;
    buck.status.current_state = BuckConverterState_e::STARTING;
}

void BuckConverterInterface::_updatePowerGood(BuckConverter_s &buck, time_ms curr_millis)
{
    const bool is_pg_high = _isPowerGoodPinSet(buck);
    if (is_pg_high != buck.is_pg_raw_high)
    {
        buck.is_pg_raw_high = is_pg_high;           // Pin flipped: remember the new level
        buck.pg_raw_changed_at_ms = curr_millis;    // and restart the debounce timer
    }
    if ((curr_millis - buck.pg_raw_changed_at_ms) >= buck.timing_params.pg_debounce_duration_ms)
    {
        buck.status.is_pg_high = is_pg_high;        // Held long enough: accept it
    }
}

void BuckConverterInterface::_enterFault(BuckConverter_s &buck, BuckConverterFaultType_e fault)
{
    buck.status.current_fault = fault;
    buck.status.current_state = BuckConverterState_e::FAULT;
    buck.status.fault_count++;
}