#ifndef FLOWMETERINTERFACE_H
#define FLOWMETERINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <Arduino.h>


namespace default_flowmeter_params
{
    const float MS_TO_S = 1000.F;
    const float PULSE_FREQ_TO_GPM = 0.0183F;
}

class FlowmeterInterface
{
public:

    FlowmeterInterface(const size_t flowmeter_pin);

    static void count_pulse() { etl::singleton<FlowmeterInterface>::instance()._pulse_count++; };

    float get_flow_gpm(unsigned long curr_millis);

private:

    unsigned long _last_sample_timestamp_ms;
    unsigned long _pulse_count = 0;
    size_t _pin;
    
};

using FlowmeterInterfaceInstance = etl::singleton<FlowmeterInterface>;

#endif // FLOWMETERINTERFACE_H
