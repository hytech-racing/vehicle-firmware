#ifndef CHARGERINTERFACE_H
#define CHARGERINTERFACE_H

/* ETL Library Includes */
#include <etl/delegate.h>
#include <etl/singleton.h>

/* External Dependencies */
#include "CANInterface.h"
#include "SharedFirmwareTypes.h"
#include "ht_can.h"

#include <FlexCAN_T4.h>

/* Local Interface Includes */
#include "ACUInterface.hpp"

namespace default_charger_params
{
    constexpr uint8_t BYTE_SHIFT = 8;
    constexpr float DATA_SCALAR = 10.0F; // charger reports values in units of 0.1
}; // namespace default_charger_params

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
    ChargerInterface(ACUInterface& acu_interface)
        : _acu_interface(acu_interface) {};

    void
    receiveChargerCANMsg(const CAN_message_t& msg, unsigned long curr_milli, ACUInterface& acu_interface, float max_pack_voltage, float cell_cutoff_voltage);

    void enqueueChargingCANMsg(ACUInterface& acu_interface, float calculated_charge_current);

    ChargerData_s get_latest_charger_data() { return _charger_data; };

    /* Charger values are sent as high/low bytes of a 16-bit int in units of 0.1 */
    float getOutputCurrentA() const { return combineBytes(_charger_data.output_current_high, _charger_data.output_current_low); };
    float getDCOutputVoltageV() const { return combineBytes(_charger_data.output_dc_voltage_high, _charger_data.output_dc_voltage_low); };
    float getACInputVoltageV() const { return combineBytes(_charger_data.input_ac_voltage_high, _charger_data.input_ac_voltage_low); };

  private:
    static float combineBytes(uint8_t high, uint8_t low)
    {
        return static_cast<float>((high << default_charger_params::BYTE_SHIFT) | low) / default_charger_params::DATA_SCALAR;
    };

    ChargerData_s _charger_data;
    ACUInterface& _acu_interface;
};

using ChargerInterfaceInstance = etl::singleton<ChargerInterface>;

#endif /* CHARGERINTERFACE_H */
