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

    ACUCoreData_s getLastReceivedData() { return _last_recvd_data; }

    float getMinCellVoltage() { return _min_cell_voltage; }

    void receiveACUVoltages(const CAN_message_t &can_msg);

private:

    ACUCoreData_s _last_recvd_data;
    float _min_cell_voltage = 0;

};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif /* ACU_INTERFACE_H */