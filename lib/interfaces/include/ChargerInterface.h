#ifndef CHARGERINTERFACE_H
#define CHARGERINTERFACE_H

/* ETL Library Includes */
#include <etl/singleton.h>
#include <etl/delegate.h>

/* External Dependencies */
#include "SharedFirmwareTypes.h"
#include "hytech.h"
#include "CANInterface.h"
#include <FlexCAN_T4.h>

/* Local Interface Includes */
#include "ACUInterface.h"


struct ChargerData_s
{
    uint8_t output_dc_voltage_high;
    uint8_t output_dc_voltage_low;
    uint8_t output_current_high;
    uint8_t output_current_low;
    uint8_t flags;
    uint8_t input_ac_voltage_high;
    uint8_t input_ac_voltage_low;
};

class ChargerInterface
{
public:

    ChargerInterface(ACUInterface& acu_interface) : _acu_interface(acu_interface) {};

    void receive_charger_data_message(const CAN_message_t& msg, unsigned long curr_milli, ACUInterface& acu_interface, float max_pack_voltage, float cell_cutoff_voltage);

    void send_charger_message();

    void enqueue_charging_data(ACUInterface& acu_interface, float calculated_charge_current);

    ChargerData_s get_latest_charger_data() {return _charger_data;};

private:

    ChargerData_s _charger_data;
    ACUInterface& _acu_interface;

};

using ChargerInterfaceInstance = etl::singleton<ChargerInterface>;

#endif /* CHARGERINTERFACE_H */