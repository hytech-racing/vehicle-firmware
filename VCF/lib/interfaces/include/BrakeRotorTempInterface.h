#ifndef BRAKEROTORTEMPINTERFACE_H
#define BRAKEROTORTEMPINTERFACE_H

/* ETL Library Includes */
#include <etl/singleton.h>

/* External Includes */
#include "hytech.h"
#include <FlexCAN_T4.h>


namespace brake_rotor_temp_default_params
{
    constexpr size_t channels_within_brake_temp_sensor = 16; // TODO: check if our sensors are actually 16 channel
}

struct BrakeRotorTempData_s
{
    float max_temp;
    float avg_temp;
    std::array<float, brake_rotor_temp_default_params::channels_within_brake_temp_sensor> channel_data;
};

struct BrakeTempSensorData_s
{
    BrakeRotorTempData_s fl_sensor;
    BrakeRotorTempData_s fr_sensor;
};

/**
 * @brief Interface to receive messages from the Izze Racing IRTS-60deg-v3 sensor
*/
class BrakeRotorTempInterface
{
public:

    // default empty constructor will init state to zeros
    BrakeRotorTempInterface() {}

    /**
     * @brief Retrieves the latest data that has been sent from the sensors
     * @return the latest temp data
    */
    BrakeTempSensorData_s getBrakeRotorTempData() const;

    /**
     * @brief CAN receive function to parse the new CAN msg and update internal state
     * @param msg the CAN msg to parse
    */
    void receiveBrakeRotorTempCANMsgs(const CAN_message_t &msg);

private:

    BrakeTempSensorData_s _temp_data;

    /**
     * @brief Helper method to update the calculated values of max and avg temps for each sensor after new data is received
     *        Will only update for the specified sensor
     * @param sensor corresponds to which sensor was updated. FL = 0, FR = 1
     */
    void _updateCalculatedValues(bool sensor);
};

using BrakeRotorTempInterfaceInstance = etl::singleton<BrakeRotorTempInterface>;

#endif // BRAKEROTORTEMP_H