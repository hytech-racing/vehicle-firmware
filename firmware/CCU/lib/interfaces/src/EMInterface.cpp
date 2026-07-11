#include "EMInterface.h"

/* External Dependencies */
#include "hytech.h"


void EnergyMeterInterface::receive_energy_meter_message(const CAN_message_t& msg, unsigned long curr_millis)
{
    EM_MEASUREMENT_t em_measurement = {};
    Unpack_EM_MEASUREMENT_hytech(&em_measurement, &msg.buf[0], msg.len);

    // as of right now the EM is flipped (6/1/25)
    _em_data.current_amps = -1.0f * static_cast<float>(HYTECH_em_current_ro_fromS(em_measurement.em_current_ro));
    _em_data.voltage = HYTECH_em_voltage_ro_fromS(em_measurement.em_voltage_ro);
}