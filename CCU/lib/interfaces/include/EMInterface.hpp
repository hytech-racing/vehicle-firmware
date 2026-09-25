#ifndef EMINTERFACE
#define EMINTERFACE

/* ETL Library */
#include <etl/singleton.h>
#include <etl/delegate.h>

/* External Includes */
#include "hytech.h"
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

    void receiveEMMeasurmentCANMsg(const CAN_message_t& msg, unsigned long curr_millis);

    EMData_s getLatestEMData() {return _em_data;};

private:

    EMData_s _em_data;

};

using EnergyMeterInterfaceInstance = etl::singleton<EnergyMeterInterface>;

#endif /* EMINTERFACE */