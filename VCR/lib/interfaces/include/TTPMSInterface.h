#ifndef TTPMSINTERFACE_H
#define TTPMSINTERFACE_H

/* Standard Library */
#include <stdint.h>

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <FlexCAN_T4.h>
#include "shared_types.h"
#include "VCRCANInterfaceImpl.h"
#include "hytech.h"

#define TEMP_CHANNELS 16

struct TTPMSSingleSensorData_s
{
    uint16_t serial_number;
    uint16_t bat_voltage;
    float pressure;
    float gauge_pressure;
    std::array<float, TEMP_CHANNELS> temp_data;
};

struct TTPMSAllSensorData_s
{
    TTPMSSingleSensorData_s fl_ttpms;
    TTPMSSingleSensorData_s fr_ttpms;
    TTPMSSingleSensorData_s rl_ttpms;
    TTPMSSingleSensorData_s rr_ttpms;
};

class TTPMSInterface
{
public:

    TTPMSInterface() {};

    /**
     * Retrieves the latest data that has been sent from the sensors
     * @return the latest temp data
     */
    TTPMSAllSensorData_s getTTPMSData() const;

    /**
     * @brief CAN receive function to parse the new CAN msg and update internal state
     * @param msg the CAN msg to parse
    */
    void receiveTTPMSData(const CAN_message_t &msg);

    /**
     * @brief Method to print all TTMPS data for a single wheel
     * @param wheel_label is a string (i.e. FL, FR, etc.)
     * @param sensor is a reference to data struct for a sensor
    */
    void printTTPMSData(const char* wheel_label, const TTPMSSingleSensorData_s &sensor);

private:

    TTPMSAllSensorData_s _ttpms_data;

};

using TTPMSInterfaceInstance = etl::singleton<TTPMSInterface>;

#endif // __TTPMSINTERFACE_H__