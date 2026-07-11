#include "ACUInterface.h"
#include "CCUCANInterfaceImpl.h" // this needs to fixed at some point


void ACUInterface::reset_acu_heartbeat()
{
    _curr_data.heartbeat_ok = true;
}

void ACUInterface::set_is_charging_enabled(bool state)
{
    _curr_data.is_charging_enabled = state;
}

void ACUInterface::receive_status_message(const CAN_message_t &msg, unsigned long curr_millis)
{
    BMS_STATUS_t bms_status_msg;
    Unpack_BMS_STATUS_hytech(&bms_status_msg, &msg.buf[0], msg.len);
    _curr_data.acu_state = static_cast<ACUState_e>(bms_status_msg.acu_state);

    // As long as we're using millis() function, loop overrun not a concern
    if(_curr_data.last_recv_status_ms == 0)
    {
        _first_received_message_heartbeat_init = true;
    }

    _curr_data.last_recv_status_ms = curr_millis;
}

void ACUInterface::receive_voltages_message(const CAN_message_t& msg, unsigned long curr_millis)
{
    BMS_VOLTAGES_t voltages_msg;
    Unpack_BMS_VOLTAGES_hytech(&voltages_msg, &msg.buf[0], msg.len);
    _curr_data.average_voltage = HYTECH_average_cell_voltage_ro_fromS(static_cast<float>(voltages_msg.average_cell_voltage_ro));
    _curr_data.low_voltage = HYTECH_min_cell_voltage_ro_fromS(static_cast<float>(voltages_msg.min_cell_voltage_ro));
    _curr_data.high_voltage = HYTECH_max_cell_voltage_ro_fromS(static_cast<float>(voltages_msg.max_cell_voltage_ro));
    _curr_data.pack_voltage = HYTECH_total_pack_voltage_ro_fromS(static_cast<float>(voltages_msg.total_pack_voltage_ro));
}

void ACUInterface::receive_detailed_voltages_message(const CAN_message_t& msg, unsigned long curr_millis)
{
    BMS_DETAILED_VOLTAGES_t voltages_msg;
    Unpack_BMS_DETAILED_VOLTAGES_hytech(&voltages_msg, &msg.buf[0], msg.len);
    uint8_t group_id = voltages_msg.group_id;
    uint8_t ic_id = voltages_msg.ic_id;
    size_t cell_base_index = (ic_id / 2) * default_acu_params::NUM_CELLS_PER_SEGMENT +
                            ((ic_id % 2 != 0) ? default_acu_params::NUM_CHIPS : 0) +
                            group_id * default_acu_params::NUM_DATA_PER_GROUP;

    _curr_data.cell_voltages[cell_base_index + 0] = HYTECH_voltage_0_ro_fromS(voltages_msg.voltage_0_ro);
    _curr_data.cell_voltages[cell_base_index + 1] = HYTECH_voltage_1_ro_fromS(voltages_msg.voltage_1_ro);
    _curr_data.cell_voltages[cell_base_index + 2] = HYTECH_voltage_2_ro_fromS(voltages_msg.voltage_2_ro);
}

void ACUInterface::receive_onboard_temps_message(const CAN_message_t& msg, unsigned long curr_millis)
{
    BMS_TEMPS_t board_temps;
    Unpack_BMS_TEMPS_hytech(&board_temps, &msg.buf[0], msg.len);
    _curr_data.max_board_temp = HYTECH_max_board_temp_ro_fromS(board_temps.max_board_temp_ro);
    _curr_data.min_cell_temp = HYTECH_min_cell_temp_ro_fromS(board_temps.min_cell_temp_ro);
    _curr_data.max_cell_temp = HYTECH_max_cell_temp_ro_fromS(board_temps.max_cell_temp_ro);
}

void ACUInterface::receive_detailed_temps_message(const CAN_message_t& msg, unsigned long curr_millis)
{
    BMS_DETAILED_TEMPS_t detailed_temps;
    Unpack_BMS_DETAILED_TEMPS_hytech(&detailed_temps, &msg.buf[0], msg.len);
    uint8_t group_id = detailed_temps.group_id;
    uint8_t ic_id = detailed_temps.ic_id;
    size_t cell_base_index = ic_id * default_acu_params::NUM_CELL_TEMPS_PER_CHIP +
                            group_id * default_acu_params::NUM_DATA_PER_GROUP;

    if (cell_base_index >= default_acu_params::NUM_CELL_TEMPS)
    {
        return;
    }

    _curr_data.cell_temps[cell_base_index] = HYTECH_thermistor_id_0_ro_fromS(detailed_temps.thermistor_id_0_ro);

    if (group_id != 1) // last group of each chip only has 1 thermistor
    {
        _curr_data.cell_temps[cell_base_index + 1] = HYTECH_thermistor_id_1_ro_fromS(detailed_temps.thermistor_id_1_ro);
        _curr_data.cell_temps[cell_base_index + 2] = HYTECH_thermistor_id_2_ro_fromS(detailed_temps.thermistor_id_2_ro);
    }
}

void ACUInterface::receive_onboard_detailed_temps(const CAN_message_t& msg, unsigned long curr_millis)
{
    BMS_BOARD_DETAILED_TEMPS_t onboard_detailed_temps{};
    Unpack_BMS_BOARD_DETAILED_TEMPS_hytech(&onboard_detailed_temps, &msg.buf[0], msg.len);
    uint8_t ic_id = onboard_detailed_temps.ic_id;
    _curr_data.board_temps[ic_id] = HYTECH_temp_0_ro_fromS(onboard_detailed_temps.temp_0_ro);
}

void ACUInterface::receive_state_of_charge(const CAN_message_t& msg, unsigned long curr_millis)
{
    STATE_OF_CHARGE_t soc_msg{};
    Unpack_STATE_OF_CHARGE_hytech(&soc_msg, &msg.buf[0], msg.len);
    _curr_data.SoC = HYTECH_SoC_ro_fromS(soc_msg.SoC_ro);
}

void ACUInterface::enqueue_ccu_status_data()
{
    CCU_STATUS_t ccu_status = {};
    ccu_status.charger_enabled = _curr_data.is_charging_enabled;
    CAN_util::enqueue_msg(&ccu_status, &Pack_CCU_STATUS_hytech, CCUCANInterfaceInstance::instance().acu_can_tx_buffer);
}