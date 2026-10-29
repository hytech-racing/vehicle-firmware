#ifndef ACU_INTERFACE
#define ACU_INTERFACE

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
    bool is_latch_ok;
    uint64_t last_acu_ok_recv_millis;

    float em_current;
    float em_voltage;

    float pack_voltage;
    uint64_t last_bms_voltages_recv_millis;

    bool is_acu_ok_heartbeat_ok;             // holds/latches false on the first missed ACU_OK, drives SOFTWARE_OK
    bool is_bms_voltages_heartbeat_ok;       // does not latch, true whenever BMS_VOLTAGES is recent
};

class ACUInterface
{
public:

    ACUInterface() = delete;

    ACUInterface(uint32_t max_acu_ok_heartbeat_interval_ms,
                uint32_t max_bms_voltages_heartbeat_interval_ms
    ) : _max_acu_ok_heartbeat_interval_ms(max_acu_ok_heartbeat_interval_ms),
        _max_bms_voltages_heartbeat_interval_ms(max_bms_voltages_heartbeat_interval_ms)
    {}

    /**
     * @brief Method unpacks the ACU_OK CAN message, updates data, and initializes the ACU heartbeat
     * @note ACU_OK message is just BMS_OK, IMD_OK, and LATCH_OK
    */
    void receiveACUOKMessage(const CAN_message_t &msg, unsigned long curr_millis);

    /**
     * @brief Method unpacks the EM_Measurement CAN message and updates data
    */
    void receiveEMMeasurementMessage(const CAN_message_t &msg, unsigned long curr_millis);

    /**
     * @brief Method unpacks the BMS_VOLTAGES CAN message and updates the pack voltage
    */
    void receiveBMSVoltagesMessage(const CAN_message_t &msg, unsigned long curr_millis);

    /**
     * @brief Re-evaluates both heartbeats against curr_millis and returns the latest data
    */
    ACUCANInterfaceData_s getLatestData(uint64_t curr_millis);

private:

    ACUCANInterfaceData_s _curr_data = {};    // all false/zero until messages arrive
    bool _has_received_first_acu_heartbeat = false;
    const uint32_t _max_acu_ok_heartbeat_interval_ms;
    const uint32_t _max_bms_voltages_heartbeat_interval_ms;

};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif