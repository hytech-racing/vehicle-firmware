#ifndef ACUINTERFACE_H
#define ACUINTERFACE_H

/* ETL Library */
#include <etl/optional.h>
#include <etl/singleton.h>
#include <etl/delegate.h>

/* External Includes */
#include <array>
#include "SharedFirmwareTypes.h"
#include "hytech.h"
#include "CANInterface.h"
#include <FlexCAN_T4.h>


namespace default_acu_params
{
    constexpr uint8_t NUM_CELL_VOLTAGES_PER_CHIP = 12;
    constexpr uint8_t NUM_CELL_TEMPS_PER_CHIP = 4;
    constexpr uint8_t NUM_BOARD_TEMPS_PER_CHIP = 1;

    constexpr uint8_t NUM_CHIPS = 12;
    constexpr uint8_t NUM_CELLS_PER_SEGMENT = 21;
    constexpr uint8_t NUM_CELLS = 126;
    constexpr uint8_t NUM_CELL_TEMPS = 48;
    constexpr uint8_t NUM_BOARD_TEMPS = 12;
    constexpr uint8_t NUM_DATA_PER_GROUP = 3;
}

struct ACUInterfaceData_s
{
    /* ACU Status Message */
    time_ms last_recv_status_ms;
    ACUState_e acu_state;
    bool heartbeat_ok;

    /* BMS Voltages */
    volt average_voltage;
    volt low_voltage;
    volt high_voltage;
    volt pack_voltage;

    /* BMS Temps Data */
    celsius max_cell_temp;
    celsius min_cell_temp;
    celsius avg_cell_temp;
    celsius max_board_temp;

    /* BMS Detailed Data */
    std::array<etl::optional<volt>, default_acu_params::NUM_CELLS> cell_voltages;
    std::array<etl::optional<celsius>, default_acu_params::NUM_CELL_TEMPS> cell_temps;
    std::array<etl::optional<celsius>, default_acu_params::NUM_BOARD_TEMPS> board_temps;

    /* Elcon Charger Status */
    bool is_charging_enabled;
    float SoC;
};

class ACUInterface
{
public:

    ACUInterface(unsigned long init_ms, unsigned long max_heartbeat_interval_ms) : _max_heartbeat_interval_ms(max_heartbeat_interval_ms)
    {
        _curr_data.last_recv_status_ms = 0;
        _curr_data.heartbeat_ok = false; // start out false
        _curr_data.acu_state = ACUState_e::STARTUP;
        _curr_data.average_voltage = 0;
        _curr_data.low_voltage = 0;
        _curr_data.high_voltage = 0;
        _curr_data.pack_voltage = 0;
        _curr_data.max_cell_temp = 0;
        _curr_data.min_cell_temp = 100;
        _curr_data.avg_cell_temp = 0;
        _curr_data.max_board_temp = 0;
    };

    bool is_acu_heartbeat_not_ok() {return !_curr_data.heartbeat_ok; }

    void reset_acu_heartbeat();

    void set_is_charging_enabled(bool state);

    /**
     * @brief Unpacks a BMS status CAN message, updates ACU state, tracks the receive timestamp, and
     *        initializes heartbeat on the first message
     */
    void receive_status_message(const CAN_message_t& msg, unsigned long curr_millis);

    /**
     * @brief Unpacks a BMS voltages CAN message and updates the current average, low, high, and pack
     *        voltage readings
     */
    void receive_voltages_message(const CAN_message_t& msg, unsigned long curr_millis); //BMS_VOLTAGES and BMS_DETAILED_VOLTAGES

    /**
     * @brief
    */
    void receive_detailed_voltages_message(const CAN_message_t& msg, unsigned long curr_millis);

    /**
     * @brief Unpacks a BMS temp CAN message and updates the max board temp, min + max cell temps
     */
    void receive_onboard_temps_message(const CAN_message_t& msg, unsigned long curr_millis);

    /**
     * @brief
     */
    void receive_detailed_temps_message(const CAN_message_t& msg, unsigned long curr_millis);

    /**
     * @brief Unpacks a detailed BMS board temp CAN message and stores the converted temp readings into the board temps
     *        array at the index corresponding to the IC ID.
     */
    void receive_onboard_detailed_temps(const CAN_message_t& msg, unsigned long curr_millis);

    /**
     * @brief Unpacks a state of charge CAN message and updates the current SoC reading
     */
    void receive_state_of_charge(const CAN_message_t& msg, unsigned long curr_millis);

    /**
     * @brief Packages the current charging_enabled state into a CCU status CAN message and enqueues it on the ACU CAN bus
     */
    void enqueue_ccu_status_data();

    ACUInterfaceData_s get_latest_data() { return _curr_data; };

private:

    ACUInterfaceData_s _curr_data;
    unsigned long _max_heartbeat_interval_ms;
    bool _first_received_message_heartbeat_init = false;

};

using ACUInterfaceInstance = etl::singleton<ACUInterface>;

#endif /* ACUINTERFACE_H */