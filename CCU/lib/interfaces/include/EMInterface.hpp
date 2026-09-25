#ifndef EMINTERFACE
#define EMINTERFACE

/* ETL Library */
#include <etl/singleton.h>
#include <etl/delegate.h>

/* External Includes */
#include <FlexCAN_T4.h>


struct EMData_s
{
    float voltage;
    float current_amps;
};

class EnergyMeterInterface
{
public:

    EnergyMeterInterface()= default;

    void receive_energy_meter_message(const CAN_message_t& msg, unsigned long curr_millis);

    EMData_s get_latest_em_data() {return _em_data;};

private:

    EMData_s _em_data;

};

using EnergyMeterInterfaceInstance = etl::singleton<EnergyMeterInterface>;

#endif /* EMINTERFACE */