#ifndef BRAKEROTORTEMPINTERFACE_H
#define BRAKEROTORTEMPINTERFACE_H

/* ETL Library Includes */
#include <etl/singleton.h>

/* External Includes */
#include <FlexCAN_T4.h>


namespace brake_rotor_temp_default_params
{
    constexpr size_t channels_within_brake_temp_sensor = 16; // TODO: check if our sensors are actually 16 channel
}

struct BrakeTempSensorData_s
{
    float max_temp;
    float avg_temp;
    std::array<float, brake_rotor_temp_default_params::channels_within_brake_temp_sensor> channel_data;
};

struct BrakeTempData_s
{
    BrakeTempSensorData_s fl_sensor;
    BrakeTempSensorData_s fr_sensor;
};

/**
 * Interface to receive messages from the Izze Racing IRTS-60deg-v3 sensor
 */
class BrakeRotorTempInterface
{
public:

// default empty constructor will init state to zeros
    BrakeRotorTempInterface() {}

    /**
     * Retrieves the latest data that has been sent from the sensors
     * @return the latest temp data
     */
    BrakeTempData_s getBrakeRotorTempData() const;

    /**
     * CAN receive function to parse the new CAN msg and update internal state
     * Called by VCF's recv switch
     * @param msg the CAN msg to parse
     */
    void receiveBrakeRotorTempData(const CAN_message_t &msg);

private:

    BrakeTempData_s _temp_data;

    void _updateCalculatedValues(bool FR);
};

using BrakeRotorTempInterfaceInstance = etl::singleton<BrakeRotorTempInterface>;

#endif // BRAKEROTORTEMP_H