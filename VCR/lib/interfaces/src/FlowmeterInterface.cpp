#include "FlowmeterInterface.h"

using namespace default_flowmeter_params;


FlowmeterInterface::FlowmeterInterface(const size_t flowmeter_pin) : _pin(flowmeter_pin), _last_sample_timestamp_ms(0UL)
{
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), count_pulse, RISING);
}

float FlowmeterInterface::get_flow_gpm(unsigned long curr_millis)
{
    if (_last_sample_timestamp_ms == 0)
    {
        return 0;
    }

    float comp_samp_frequency = MS_TO_S / static_cast<float>(curr_millis - _last_sample_timestamp_ms);
    _last_sample_timestamp_ms = curr_millis;

    float reading = PULSE_FREQ_TO_GPM * static_cast<float>(_pulse_count) * comp_samp_frequency;
    _pulse_count = 0;

    return reading;
}