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
    bool bms_ok;
    bool imd_ok;
    uint64_t last_recv_millis;

    float em_current;
    float em_voltage;

    bool heartbeat_ok;
};

class ACUInterface
{
public:

    ACUInterface() = delete;

    ACUInterface(uint32_t init_millis, uint32_t max_heartbeat_interval_ms) : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.bms_ok = _curr_data.imd_ok = false;
        _curr_data.last_recv_millis = 0;
    };

    void receive_acu_ok_message(const CAN_message_t &msg, unsigned long curr_millis);

    void receive_em_measurement(const CAN_message_t &msg, unsigned long curr_millis);

    /* Getters */
    bool is_imd_ok() { return _curr_data.imd_ok; }

    bool is_bms_ok() { return _curr_data.bms_ok; }

    uint64_t get_last_received_msg_time() { return _curr_data.last_recv_millis; }

    bool has_received_first_acu_heartbeat() { return _received_first_acu_heartbeat; }

    ACUCANInterfaceData_s get_latest_data(uint64_t curr_millis);

private:

    ACUCANInterfaceData_s _curr_data;
    bool _received_first_acu_heartbeat = false;
    const uint32_t _max_heartbeat_interval_ms;

};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif