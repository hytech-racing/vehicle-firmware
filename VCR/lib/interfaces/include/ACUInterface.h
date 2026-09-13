#ifndef ACUINTERFACE_H
#define ACUINTERFACE_H

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include "SharedFirmwareTypes.h"
#include "shared_types.h"
#include <FlexCAN_T4.h>

struct ACUCANInterfaceData_s
{
    bool is_bms_ok;
    bool is_imd_ok;
    uint64_t last_msg_recieved_millis;

    float em_current;
    float em_voltage;

    bool is_heartbeat_ok;
};

class ACUInterface
{
public:

    ACUInterface() = delete;

    ACUInterface(uint32_t init_millis,
                uint32_t max_heartbeat_interval_ms
    ) : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.is_bms_ok = false;
        _curr_data.is_imd_ok = false;
        _curr_data.last_msg_recieved_millis = 0;
    };

    /**
     * @brief Method unpacks the ACU_OK CAN message, updates data, and initializes the ACU heartbeat
     * @note ACU_OK message is just BMS_OK and IMD_OK
    */
    void receiveACUOKMessage(const CAN_message_t &msg, unsigned long curr_millis);

    /**
     * @brief Method unpacks the EM_Measurement CAN message and updates data
    */
    void receiveEMMeasurementMessage(const CAN_message_t &msg, unsigned long curr_millis);

    bool isIMDOk()
    {
        return _curr_data.is_imd_ok;
    }

    bool isBMSOk()
    {
        return _curr_data.is_bms_ok;
    }

    uint64_t getLastReceivedMsgMillis()
    {
        return _curr_data.last_msg_recieved_millis;
    }

    bool hasReceivedFirstACUHeartbeat()
    {
        return _has_received_first_acu_heartbeat;
    }

    ACUCANInterfaceData_s getLatestData(uint64_t curr_millis);

private:

    ACUCANInterfaceData_s _curr_data;
    bool _has_received_first_acu_heartbeat = false;
    const uint32_t _max_heartbeat_interval_ms;

};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif