#ifndef ACU_INTERFACE_H
#define ACU_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech.h"

/* Local Interface Includes */
#include "CANInterface.h"
#include "SystemTimeInterface.h"

struct ACUData_s
{
    bool bms_ok;
    bool imd_ok;
    volt pack_voltage;
    volt min_cell_voltage;
    float soc_percent; // 0-100, negative until the first STATE_OF_CHARGE message arrives
    float soe_percent; // 0-100, negative until the first STATE_OF_ENERGY message arrives
};

class ACUInterface
{
public:

    //ACUInterface() = delete;

    ACUInterface()
    {
        _acu_data.bms_ok = true;
        _acu_data.imd_ok = true;
        _acu_data.pack_voltage = 460;
        _acu_data.min_cell_voltage = 0;
        _acu_data.soc_percent = -1.0f;
        _acu_data.soe_percent = -1.0f;
    }

    void receive_acu_ok_message(const CAN_message_t &msg);

    ACUData_s get_curr_data() {return _acu_data;}

    void receive_acu_voltages(const CAN_message_t &msg);

    void receive_state_of_charge(const CAN_message_t &msg);

    void receive_state_of_energy(const CAN_message_t &msg);


private:

    ACUData_s _acu_data;

};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif /* ACU_INTERFACE_H */