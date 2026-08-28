#ifndef TTPMSINTERFACE_H
#define TTPMSINTERFACE_H

/* Standard Library */
#include <stdint.h>

/* External Includes */
#include "shared_types.h"
#include "FlexCAN_T4.h"
#include <CANInterface.h>
#include <hytech.h>

#define TEMP_CHANNELS 16

struct TTPMSSingleSensorData_s
{ 
    uint16_t bat_voltage;
    uint16_t pressure;
    uint16_t gauge_pressure;
    std::array<float, TEMP_CHANNELS> temp_data;
};

struct TTPMSAllSensorData_s
{
    TTPMSSingleSensorData_s lf_ttpms;
    TTPMSSingleSensorData_s rf_ttpms;
    TTPMSSingleSensorData_s lr_ttpms;
    TTPMSSingleSensorData_s rr_ttpms;
};

/**
 * TTPMS interface
 */
class TTPMSInterface
{
public:
    TTPMSInterface() {};

    /**
     * Retrieves the latest data that has been sent from the sensors
     * @return the latest temp data
     */
    TTPMSAllSensorData_s get_ttpms_data() const;

    // TODO: update VCRCANInterface to receive messages from TTPMS
    /**
     * CAN receive function to parse the new CAN msg and update internal state
     * Called by VCR's recv switch
     * @param msg the CAN msg to parse
     */
    void receive_ttpms_data(const CAN_message_t &msg);
private:
    TTPMSAllSensorData_s _ttpms_data;

}

#endif // __TTPMSINTERFACE_H__