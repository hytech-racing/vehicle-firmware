#ifndef ACU_INTERFACE_H
#define ACU_INTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "hytech.h"
#include "CANInterface.h"
#include <FlexCAN_T4.h>


class ACUInterface
{
public:

    ACUCoreData_s get_last_recvd_data() { return _last_recvd_data; }

    float get_cell_voltage() { return _min_cell_voltage; }

    void receive_ACU_voltages(const CAN_message_t &can_msg);

private:

    ACUCoreData_s _last_recvd_data;
    float _min_cell_voltage = 0;

};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif /* ACU_INTERFACE_H */