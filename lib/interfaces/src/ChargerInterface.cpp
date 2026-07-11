#include "ChargerInterface.h"
#include "CCUCANInterfaceImpl.h" // this needs to fixed at some point


void ChargerInterface::receive_charger_data_message(const CAN_message_t& msg, unsigned long curr_millis, ACUInterface& acu_interface, float max_pack_voltage, float cell_cutoff_voltage)
{
    CHARGER_DATA_t charger_data_msg;
    //charger_data_s charger_data; //NOLINT - needed for initialization
    Unpack_CHARGER_DATA_hytech(&charger_data_msg, &msg.buf[0], msg.len);
    _charger_data.output_dc_voltage_high = charger_data_msg.output_dc_voltage_high;
    _charger_data.output_dc_voltage_low = charger_data_msg.output_dc_voltage_low;
    _charger_data.output_current_high = charger_data_msg.output_current_high;
    _charger_data.output_current_low = charger_data_msg.output_current_low;
    _charger_data.flags = charger_data_msg.flags;
    _charger_data.input_ac_voltage_high = charger_data_msg.input_ac_voltage_high;
    _charger_data.input_ac_voltage_low = charger_data_msg.input_ac_voltage_low;
    acu_interface.set_is_charging_enabled(true); //if a charger message is received, we are ready to start charging

    /* Redundancy to avoid flipping between true and false for balancing (charging) enabled */
    if (acu_interface.get_latest_data().pack_voltage >= max_pack_voltage || ACUInterfaceInstance::instance().get_latest_data().high_voltage >= cell_cutoff_voltage)
    {
        acu_interface.set_is_charging_enabled(false);
    }
}

void ChargerInterface::enqueue_charging_data(ACUInterface& acu_interface, float calculated_charge_current)
{
    // Charging voltage is in units of 100mV
    // So, we want 530V to be max charge voltage
    // We want the high/low bytes of the 16-bit int to be 5300 (decimal), which is 0x14B4 (hex)

    // NOTE: The "high" and "low" values are not max and min-- rather, they are the "high" and "low" bytes of a 16-bit integer.
    CHARGER_CONTROL_t charger_control = {};
    charger_control.max_charging_voltage_high = 0x14; //NOLINT (see comment) - need to change this in PCAN library
    charger_control.max_charging_voltage_low = 0xB4; //NOLINT (see comment)
    charger_control.max_charging_current_high = 0; // only "low" is being used/harnessed in
    charger_control.max_charging_current_low = static_cast<uint8_t>(calculated_charge_current * 10); //NOLINT (this works)
    CAN_util::enqueue_msg(&charger_control, &Pack_CHARGER_CONTROL_hytech, CCUCANInterfaceInstance::instance().charger_can_tx_buffer);
}
