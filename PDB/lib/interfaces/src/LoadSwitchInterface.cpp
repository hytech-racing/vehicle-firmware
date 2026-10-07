#include "LoadSwitchInterface.h"

LoadSwitchParams_s LDSWs_params[6] = {
    LoadSwitchParams_s(PE4, PE5, PB0, 100000, 20000, ), //Camera
    LoadSwitchParams_s(PE6, PE7, PB1, 31600, 4870, ), //Orin
    LoadSwitchParams_s(PE8, PE9, PC4, 22100, 5100, ), //Inverter
    LoadSwitchParams_s(PE10, PE11, PC5, 22100, 5100, ), //Motor
    LoadSwitchParams_s(PE12, PE13, PA6, 84500, 14700, ), //Lidar
    LoadSwitchParams_s(PE14, PE15, PA7, 24900, 3900, ) //DTI
};

void initLoadSwitches() {
    for (int i = 0; i < 6; i++) {
        LDSWs[i] = LoadSwitchInterface(LDSWs_params[i]);
        LDSWs[i].init();
    }
}

void LoadSwitchInterface::init()
{
    // Write the output latch LOW *before* making the pin an output so SHDN never
    // glitches high at boot.
    digitalWrite(_params.enable_pin, LOW);
    pinMode(_params.enable_pin, OUTPUT);

    // FLT is an open-drain output from the eFuse with an external pull-up on the
    // board, so the MCU side is a plain input (no internal pull needed).
    pinMode(_params.fault_pin, INPUT);

    // IMON: analog input; analogRead() configures the pin on STM32duino.

    _current_data.is_loadswitch_enabled = false;
    _current_data.is_loadswitch_faulted = false;
    _current_data.imon_current_mA       = 0.0f;
}

void LoadSwitchInterface::enable()
{
    // Only restart the ignore fault timer on an off -> on transition
    if (!_current_data.is_loadswitch_enabled) {
        _enable_time_ms = millis();
    }

    // SHDN > V(SHUTR) = 2 V enables the device; output then ramps in dVdT mode (8.3.13).
    digitalWrite(_params.enable_pin, HIGH);
    _current_data.is_loadswitch_enabled = true;
}

void LoadSwitchInterface::disable()
{
    // SHDN < V(SHUTF) = 0.8 V turns the FET off within tSD(dly) = 1 us typ (6.6).
    digitalWrite(_params.enable_pin, LOW);
    _current_data.is_loadswitch_enabled = false;
    _current_data.is_loadswitch_faulted = false;
}

void LoadSwitchInterface::reset_latched_fault()
{
    /*
     * MODE pin open = latch-off mode (Table 8-1). After an overload / thermal fault the
     * FET stays off until SHDN is toggled low -> high, UVLO is cycled, or IN_SYS is
     * power-cycled.
     *
     * Low pulse width must exceed tSD(dly) max = 1.5 us (6.6); LATCH_RESET_LOW_US gives
     * plenty of margin.
     */
    digitalWrite(_params.enable_pin, LOW);
    delayMicroseconds(loadswitch_default_params::LATCH_RESET_LOW_US);
    digitalWrite(_params.enable_pin, HIGH);

    _current_data.is_loadswitch_enabled = true;
    _current_data.is_loadswitch_faulted = false;
    _enable_time_ms        = millis(); // output restarts with a full dVdT ramp
}

void LoadSwitchInterface::sampleFault()
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
    if (!_current_data.is_loadswitch_enabled ||
        (millis() - _enable_time_ms) < _params.startup_ignore_fault_ms)
    {
        _current_data.is_loadswitch_faulted = false;
        return;
    }

    // Active-low: LOW = fault (UV, OV, overload, reverse current, ILIM open/short, TSD).
    _current_data.is_loadswitch_faulted = (digitalRead(_params.fault_pin) == LOW);
}

void LoadSwitchInterface::sampleIMON()
{
    /**
     * @warning Single-sample read. IMON is a high-impedance source (R_IMON, tens of kOhm)
     *          and must not have a bypass cap (8.3.9). If readings are noisy consider a
     *          discarded first read after channel switch, averaging/IIR filtering, etc.
     *
     * I_OUT[mA] = counts * mA_per_count       (mA_per_count from _computeMilliampsPerCount)
     */
    const auto counts = static_cast<uint16_t>(analogRead(_params.imon_pin));
    _current_data.imon_current_mA  = _convertCountsTomA(counts, _imon_mA_per_count);
}

float LoadSwitchInterface::imon_full_scale_mA() const
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
    const float v_max = (loadswitch_default_params::ADC_VREF_V < loadswitch_default_params::IMON_MAX_VOLTAGE_UV)
                            ? loadswitch_default_params::ADC_VREF_V
                            : loadswitch_default_params::IMON_MAX_VOLTAGE_UV;

    const float max_counts = (v_max / loadswitch_default_params::ADC_VREF_V)
                             * static_cast<float>((1UL << loadswitch_default_params::ADC1_RESOLUTION) - 1UL);

    return _convertCountsTomA(static_cast<uint16_t>(max_counts), _imon_mA_per_count);
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