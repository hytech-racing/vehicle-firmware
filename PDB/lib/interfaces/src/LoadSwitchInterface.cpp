#include "LoadSwitchInterface.hpp"


LoadSwitchInterface::LoadSwitchInterface(const std::array<LoadSwitchParams_s, NUM_LOAD_SWITCHES> &params)
{
    bool is_filled[NUM_LOAD_SWITCHES] = {};

    for (const LoadSwitchParams_s &switch_params : params)
    {
        // Place by id so the list order doesn't matter; a bad or repeated id invalidates the whole config
        if (!_isValidIndex(switch_params.id) || is_filled[switch_params.id])
        {
            _is_config_valid = false;
            continue;
        }
        is_filled[switch_params.id] = true;

        LoadSwitch_s &load_switch = _all_load_switches[switch_params.id];
        load_switch.params = switch_params;
        load_switch.imon_mA_per_count = _computeMilliampsPerCount(switch_params.imon_resistor_ohms);
        // I_OL[A] = 18 / R_ILIM[kOhm]  ->  I_OL[mA] = 18 * 10^6 / R_ILIM[Ohm]
        load_switch.current_limit_mA = switch_params.ilim_resistor_ohms
            ? (loadswitch_default_params::OVERLOAD_CURRENT_NUMERATOR * 1'000'000UL) / switch_params.ilim_resistor_ohms
            : 0;
    }
}

bool LoadSwitchInterface::init()
{
    if (!_is_config_valid)
    {
        return false;
    }

    for (LoadSwitch_s &load_switch : _all_load_switches)
    {
        // Write the output latch LOW *before* making the pin an output so SHDN never
        // glitches high at boot.
        digitalWrite(load_switch.params.enable_pin, LOW);
        pinMode(load_switch.params.enable_pin, OUTPUT);

        // FLT is an open-drain output from the eFuse with an external pull-up on the
        // board, so the MCU side is a plain input (no internal pull needed).
        pinMode(load_switch.params.fault_pin, INPUT);

        // IMON: analog input; analogRead() configures the pin on STM32duino.

        load_switch.data = {};
    }
    return true;
}

void LoadSwitchInterface::enable(uint8_t index)
{
    if (!_isValidIndex(index))
    {
        return;
    }
    LoadSwitch_s &load_switch = _all_load_switches[index];

    // Only restart the ignore fault timer on an off -> on transition
    if (!load_switch.data.is_loadswitch_enabled)
    {
        load_switch.enable_time_ms = millis();
    }

    // SHDN > V(SHUTR) = 2 V enables the device; output then ramps in dVdT mode (8.3.13).
    digitalWrite(load_switch.params.enable_pin, HIGH);
    load_switch.data.is_loadswitch_enabled = true;
}

void LoadSwitchInterface::disable(uint8_t index)
{
    if (!_isValidIndex(index))
    {
        return;
    }
    LoadSwitch_s &load_switch = _all_load_switches[index];

    // SHDN < V(SHUTF) = 0.8 V turns the FET off within tSD(dly) = 1 us typ (6.6).
    digitalWrite(load_switch.params.enable_pin, LOW);
    load_switch.data.is_loadswitch_enabled = false;
    load_switch.data.is_loadswitch_faulted = false;
}

void LoadSwitchInterface::enableAll()
{
    for (uint8_t i = 0; i < NUM_LOAD_SWITCHES; i++)
    {
        enable(i);
    }
}

void LoadSwitchInterface::disableAll()
{
    for (uint8_t i = 0; i < NUM_LOAD_SWITCHES; i++)
    {
        disable(i);
    }
}

void LoadSwitchInterface::reset_latched_fault(uint8_t index)
{
    /*
     * MODE pin open = latch-off mode (Table 8-1). After an overload / thermal fault the
     * FET stays off until SHDN is toggled low -> high, UVLO is cycled, or IN_SYS is
     * power-cycled.
     *
     * Low pulse width must exceed tSD(dly) max = 1.5 us (6.6); LATCH_RESET_LOW_US gives
     * plenty of margin.
     */
    if (!_isValidIndex(index))
    {
        return;
    }
    LoadSwitch_s &load_switch = _all_load_switches[index];

    digitalWrite(load_switch.params.enable_pin, LOW);
    delayMicroseconds(loadswitch_default_params::LATCH_RESET_LOW_US);
    digitalWrite(load_switch.params.enable_pin, HIGH);

    load_switch.data.is_loadswitch_enabled = true;
    load_switch.data.is_loadswitch_faulted = false;
    load_switch.enable_time_ms = millis(); // output restarts with a full dVdT ramp
}

void LoadSwitchInterface::sampleFault(uint8_t index)
{
    /*
     * FLT is only meaningful while enabled and after start-up has finished.
     * startup_ignore_fault_ms should cover turn-on delay + output ramp:
     *
     *   t_on    = 742 us + 49.5 us * C_dVdT[nF]          (UVLO_ton(dly), 6.6)
     *   t_dVdT  = 20.8e3 * V_IN * C_dVdT                 (Eq. 2, 8.3.1)
     *   t_blank >= t_on + t_dVdT
     *
     * e.g. C_dVdT = 100 nF, V_IN = 24 V:
     *   t_on = 5.7 ms, t_dVdT = 49.9 ms  ->  ~56 ms, so use ~100 ms.
     */
    if (!_isValidIndex(index))
    {
        return;
    }
    LoadSwitch_s &load_switch = _all_load_switches[index];

    if (!load_switch.data.is_loadswitch_enabled ||
        (millis() - load_switch.enable_time_ms) < load_switch.params.startup_ignore_fault_ms)
    {
        load_switch.data.is_loadswitch_faulted = false;
        return;
    }

    // Active-low: LOW = fault (UV, OV, overload, reverse current, ILIM open/short, TSD).
    load_switch.data.is_loadswitch_faulted = (digitalRead(load_switch.params.fault_pin) == LOW);
}

void LoadSwitchInterface::sampleIMON(uint8_t index)
{
    /**
     * @warning Single-sample read. IMON is a high-impedance source (R_IMON, tens of kOhm)
     *          and must not have a bypass cap (8.3.9). If readings are noisy consider a
     *          discarded first read after channel switch, averaging/IIR filtering, etc.
     *
     * I_OUT[mA] = counts * mA_per_count       (mA_per_count from _computeMilliampsPerCount)
     */
    if (!_isValidIndex(index))
    {
        return;
    }
    LoadSwitch_s &load_switch = _all_load_switches[index];

    const auto counts = static_cast<uint16_t>(analogRead(load_switch.params.imon_pin));
    load_switch.data.imon_current_mA = _convertCountsTomA(counts, load_switch.imon_mA_per_count);
}

void LoadSwitchInterface::sampleAll()
{
    for (uint8_t i = 0; i < NUM_LOAD_SWITCHES; i++)
    {
        sampleFault(i);
        sampleIMON(i);
    }
}

const LoadSwitchData_s *LoadSwitchInterface::getData(uint8_t index) const
{
    return _isValidIndex(index) ? &_all_load_switches[index].data : nullptr;
}

bool LoadSwitchInterface::isLoadSwitchEnabled(uint8_t index) const
{
    return _isValidIndex(index) && _all_load_switches[index].data.is_loadswitch_enabled;
}

bool LoadSwitchInterface::isLoadSwitchFaulted(uint8_t index) const
{
    return _isValidIndex(index) && _all_load_switches[index].data.is_loadswitch_faulted;
}

uint32_t LoadSwitchInterface::getIMONCurrent(uint8_t index) const
{
    return _isValidIndex(index) ? _all_load_switches[index].data.imon_current_mA : 0;
}

uint32_t LoadSwitchInterface::current_limit_mA(uint8_t index) const
{
    return _isValidIndex(index) ? _all_load_switches[index].current_limit_mA : 0;
}

bool LoadSwitchInterface::anyFault() const
{
    for (const LoadSwitch_s &load_switch : _all_load_switches)
    {
        if (load_switch.data.is_loadswitch_faulted)
        {
            return true;
        }
    }
    return false;
}

float LoadSwitchInterface::imon_full_scale_mA(uint8_t index) const
{
    /*
     * Highest load current IMON can report. Limited by whichever is lower:
     *   - ADC reference V_REF
     *   - IMON's 4 V maximum monitoring voltage (8.3.9, 6.3)
     *
     *   V_max     = min(V_REF, 4 V)
     *   counts_FS = (V_max / V_REF) * (2^N - 1)
     *   I_FS[mA]  = counts_FS * mA_per_count
     *
     * Sanity check at init: I_FS should be >= 2 * I_OL so the 2x pulse current
     * (TPS26631, 8.3.7.1.2) doesn't saturate the ADC.
     */
    if (!_isValidIndex(index))
    {
        return 0.0f;
    }

    // Compare in volts: IMON_MAX_VOLTAGE_UV is in microvolts
    const float imon_max_v = static_cast<float>(loadswitch_default_params::IMON_MAX_VOLTAGE_UV) / 1'000'000.0f;
    const float v_max = (loadswitch_default_params::ADC_VREF_V < imon_max_v)
                            ? loadswitch_default_params::ADC_VREF_V
                            : imon_max_v;

    const float max_counts = (v_max / loadswitch_default_params::ADC_VREF_V)
                             * static_cast<float>((1UL << loadswitch_default_params::ADC1_RESOLUTION) - 1UL);

    return _convertCountsTomA(static_cast<uint16_t>(max_counts), _all_load_switches[index].imon_mA_per_count);
}

float LoadSwitchInterface::_convertCountsTomA(uint16_t counts, float mA_per_count)
{
    return static_cast<float>(counts) * mA_per_count;
}

float LoadSwitchInterface::_computeMilliampsPerCount(uint32_t imon_resistor_ohms)
{
    /**
     *   V_IMON = I_OUT * GAIN_IMON * R_IMON
     *
     *   V_IMON = counts * V_REF / 2^N
     *
     *   I_OUT[A] = counts * (V_REF / 2^N) / (GAIN_IMON * R_IMON)
     *
     * Constant part, converted to mA:
     *   mA_per_count = (V_REF / 2^N) / (R_IMON * GAIN_IMON) * 1000
    */
    if (imon_resistor_ohms == 0)
    {
        return 0.0f;
    }

    const float volts_per_count = loadswitch_default_params::ADC_VREF_V
                                / static_cast<float>(1UL << loadswitch_default_params::ADC1_RESOLUTION);

    return volts_per_count
           / (static_cast<float>(imon_resistor_ohms) * loadswitch_default_params::IMON_GAIN)
           * 1000.0f;
}
