#ifndef PEDALSSYSTEM
#define PEDALSSYSTEM

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <math.h>
#include <tuple>
#include "SharedFirmwareTypes.h"


const int IMPLAUSIBILITY_DURATION = 100; // max duration of implausibility in milliseconds. FSAE Rules T.4.3.3
const float IMPLAUSIBILITY_PERCENT = static_cast<float>(0.10); // 10 percent implausibility margin. FSAE Rules T.4.2.5
const float ACCELERATION_PERCENT_LIMIT = static_cast<float>(0.05); // acceleration percent limit to start implausibility time

/**
 * Pedals params struct that holds min/max values that will be used for evaluation. The accel and brake sensors
 * will each have one version of PedalsParam.
 *
 * NOTE: Please take note of the meaning of min/max! They are not the min/max outputs of the pedal sensor, they
 *       the values at min/max travel. So, for negative slope coefficient sensors, "min" will be greater than "max".
 */
struct PedalsParams
{
    uint32_t min_pedal_1; // Sensor 1 value at min pedal travel (analog 0-4095)
    uint32_t min_pedal_2; // Sensor 2 value at min pedal travel (analog 0-4095)
    uint32_t max_pedal_1; // Sensor 1 value at max pedal travel (analog 0-4095)
    uint32_t max_pedal_2; // Sensor 2 value at max pedal travel (analog 0-4095)
    float activation_percentage; // Percent value (range from 0.0 to 1.0)
    uint32_t min_sensor_pedal_1; // Min value that the sensor can output (if ADC reads less than this, sensor is likely unplugged)
    uint32_t min_sensor_pedal_2; // Min value that the sensor can output (if ADC reads less than this, sensor is likely unplugged)
    uint32_t max_sensor_pedal_1; // Max value that the sensor can output (if ADC reads more than this, sensor is likely unplugged)
    uint32_t max_sensor_pedal_2; // Max value that the sensor can output (if ADC reads more than this, sensor is likely unplugged)
    float deadzone_margin; // "Deadzone" margin on each side, in percent. (i.e. if deadzone_margin = 0.05, then the range 0.05-0.95 will be scaled up to 0.0-1.0)
    float implausibility_margin; // Out-of-range implausibility margin (0.0 to 1.0). If margin is 0.10, then the pedal can travel 10% beyond the measured min/max
                                 // before triggering an OOR implausibility.
    float mechanical_activation_percentage; // Ranges from 0.0 to 1.0. For brake pedal, this is the percent at which the mechanical brake engages.
};


/**
 * The PedalsSystem is responsible for taking in the analog values for the brake/pedals and reporting the
 * most up-to-date PedalsSystemData_s. See definition of PedalsSystem data for the meaning of each signal.
 */
class PedalsSystem
{
public:

    /// @param accelParams Accel pedal parameters. By rules, 2 sensors must be used for redundancy and evaluated w.r.t each other
    /// @param brakeParams Brake pedal params. When used with only one pedal sensor, the pedal parameter evaluation for brakes only looks at the min and max for min_pedal_1 / max_pedal_1
    PedalsSystem(const PedalsParams &accelParams, const PedalsParams &brakeParams) :
        _accelParams(accelParams),
        _brakeParams(brakeParams),
        _implausibilityStartTime(0)
    {};

    void set_params(const PedalsParams &accelParams, const PedalsParams &brakeParams)
    {
        _accelParams = accelParams;
        _brakeParams = brakeParams;

    }

    void set_pedals_sensor_data(const PedalSensorData_s &pedal_data)
    {
        _sensorData.accel_1 = pedal_data.accel_1;
        _sensorData.accel_2 = pedal_data.accel_2;
        _sensorData.brake_1 = pedal_data.brake_1;
        _sensorData.brake_2 = pedal_data.brake_2;
    }

    const PedalsSystemData_s &get_pedals_system_data()
    {
        return _systemData;
    }

    const PedalSensorData_s &get_pedals_sensor_data()
    {
        return _sensorData;
    }

    float get_mech_brake_activation_threshold()
    {
        return _brakeParams.mechanical_activation_percentage;
    }

    /// @brief Pedal evaluation function that takes in the direct analog values of the pedals and
    ///        returns all of the pedals system data.
    void evaluate_pedals(PedalSensorData_s pedal_data, unsigned long curr_millis);

    PedalsParams get_accel_params() {return _accelParams;}
    PedalsParams get_brake_params() {return _brakeParams;}

    /**
     * This is a way to force-update the calibrated min/max values.
     * WARNING: This should only be called when driver holds the button!
     * WARNING: This requires both pedals to be near 0% travel!
     */
    void recalibrate_min_max(const PedalSensorData_s &curr_values)
    {
        // If pedal is near 0% travel and is closer to the observed max, then this sensor is a negative coefficient.
        bool accel_1_flipped = std::abs((int) curr_values.accel_1 - (int) max_observed_accel_1) < std::abs((int) curr_values.accel_1 - (int) min_observed_accel_1);
        bool accel_2_flipped = std::abs((int) curr_values.accel_2 - (int) max_observed_accel_2) < std::abs((int) curr_values.accel_2 - (int) min_observed_accel_2);
        bool brake_1_flipped = std::abs((int) curr_values.brake_1 - (int) max_observed_brake_1) < std::abs((int) curr_values.brake_1 - (int) min_observed_brake_1);
        bool brake_2_flipped = std::abs((int) curr_values.brake_2 - (int) max_observed_brake_2) < std::abs((int) curr_values.brake_2 - (int) min_observed_brake_2);

        _accelParams.min_pedal_1 = accel_1_flipped ? max_observed_accel_1 : min_observed_accel_1;
        _accelParams.max_pedal_1 = accel_1_flipped ? min_observed_accel_1 : max_observed_accel_1;
        _accelParams.min_pedal_2 = accel_2_flipped ? max_observed_accel_2 : min_observed_accel_2;
        _accelParams.max_pedal_2 = accel_2_flipped ? min_observed_accel_2 : max_observed_accel_2;
        _brakeParams.min_pedal_1 = brake_1_flipped ? max_observed_brake_1 : min_observed_brake_1;
        _brakeParams.max_pedal_1 = brake_1_flipped ? min_observed_brake_1 : max_observed_brake_1;
        _brakeParams.min_pedal_2 = brake_2_flipped ? max_observed_brake_2 : min_observed_brake_2;
        _brakeParams.max_pedal_2 = brake_2_flipped ? min_observed_brake_2 : max_observed_brake_2;
    }

    /**
     * From code startup, the PedalsSystem should constantly update what its observed max/min
     * values are. When the driver triggers a pedal recalibration, these values will be written
     * to non-volatile memory (EEPROM). This should be called constantly to update the
     * observation.
     */
    void update_observed_pedal_limits(const PedalSensorData_s &curr_values)
    {
        min_observed_accel_1 = std::min(min_observed_accel_1, curr_values.accel_1);
        max_observed_accel_1 = std::max(max_observed_accel_1, curr_values.accel_1);
        min_observed_accel_2 = std::min(min_observed_accel_2, curr_values.accel_2);
        max_observed_accel_2 = std::max(max_observed_accel_2, curr_values.accel_2);
        min_observed_brake_1 = std::min(min_observed_brake_1, curr_values.brake_1);
        max_observed_brake_1 = std::max(max_observed_brake_1, curr_values.brake_1);
        min_observed_brake_2 = std::min(min_observed_brake_2, curr_values.brake_2);
        max_observed_brake_2 = std::max(max_observed_brake_2, curr_values.brake_2);
    }
    uint32_t min_observed_accel_1 = 4095;
    uint32_t max_observed_accel_1 = 0;
    uint32_t min_observed_accel_2 = 4095;
    uint32_t max_observed_accel_2 = 0;
    uint32_t min_observed_brake_1 = 4095;
    uint32_t max_observed_brake_1 = 0;
    uint32_t min_observed_brake_2 = 4095;
    uint32_t max_observed_brake_2 = 0;

private:
    /// @brief function to determine the percentage of pedal pressed
    /// @param pedal1val the value of the first pedal without deadzone removed (analog 0-4095)
    /// @param pedal2val the value of the second pedal without deadzone removed (analog 0-4095)
    /// @param params the pedal parameters for this specific pedal
    float _pedal_percentage(float scaled_pedal_1, float scaled_pedal_2, const PedalsParams& params);

    /// @brief function to scale the pedal value to a 0-1 value without deadzone for the first pedal
    /// @param pedalval the value of the pedal without deadzone removed (analog 0-4095)
    /// @param max_pedal pedal maximum
    /// @param min_pedal pedal minimum
    /// @return the scaled value of the pedal without deadzone removed (0-1)
    float _pedals_scaler(int pedal_val, int max_pedal, int min_pedal);

    /// @brief function to remove deadzone from pedal data
    /// @param conversion_input the value of the pedal without deadzone removed
    /// @param deadzone the deadzone value for this specific pedal
    float _remove_deadzone(float conversion_input, float deadzone);


    /// @brief function to determine if the implausibility duration has been exceeded
    /// @param curr_time the current time in milliseconds
    bool _max_duration_of_implausibility_exceeded(unsigned long curr_time);

    /// @brief
    ///    Evaluate pedal implausibilities_ determines if there is a software implausibility
    ///    in the pedals caused by them going out of range.
    ///    Our max/min sensor ranges are calcuated from the pedal min/max values
    ///    The pedal min/max values are defined in MCU_defs and are the real world analog
    ///    values that we determine from the pedal output.
    ///    The max/min sensor values are then a certain percent higher than these real world
    ///    values as determined by the implausibility margin. This protects against physical
    ///    damage to the sensor or pedal, but will not accidentally trip implausibility if
    ///    pedal values fluctuate

    /// @param pedalData1
    /// @param pedalData2
    /// @param params
    /// @param max_percent_diff
    /// @return
    bool _evaluate_pedal_implausibilities(float pedal_1_scaled,
                                          float pedal_2_scaled,
                                          int pedal_1_analog,
                                          int pedal_2_analog,
                                          const PedalsParams &params,
                                          float max_percent_diff);

    /// @brief function to determine if the pedals and the brakes are pressed at the same time.
    ///        evaluates brake being pressed with mech brake activation threshold AFTER removing
    ///        deadzones for both brake and accel
    /// @param pedal_data the pedal data struct containing the values of the pedals
    bool _evaluate_brake_and_accel_pressed(PedalSensorData_s & pedal_data);

    /// @brief function to determine if the pedal is out of range of the calibrated value for the pedal
    /// @param pedalData_analog the value of the pedal without deadzone removed
    /// @param min the min value of the pedal -- min sensor value
    /// @param max the max value of the pedal -- max sensor value
    bool _evaluate_pedal_oor(int pedalData_analog,
                           int min,
                           int max);
    /// @brief
    /// @param pedalData
    /// @param min
    /// @param max
    /// @param implaus_margin_scale
    /// @return
    bool _evaluate_min_max_pedal_implausibilities(int pedalData_analog,
                                                  int min,
                                                  int max,
                                                  float implaus_margin_scale);

    /// @brief Checks whether or not pedal is active according to input parameters. Returns true if either pedal is over threshold. Removes the deadzone before checking.
    /// @param pedal1ConvertedData the value 0 to 1 of the first pedal without deadzone removed
    /// @param pedal2ConvertedData ... second pedal 0 to 1 val
    /// @param params the pedal parameters for this specific pedal
    /// @param check_mech_activation if this is true, function will check percentages against the mechanical activation percentage
    /// @return true or false accordingly
    bool _pedal_is_active(float pedal1ConvertedData, float pedal2ConvertedData, const PedalsParams &params, bool check_mech_activation);

private:
    PedalsSystemData_s _systemData{};
    PedalSensorData_s _sensorData{};
    PedalsParams _accelParams{};
    PedalsParams _brakeParams{};
    bool _implaus_occured = false;
    unsigned long _implausibilityStartTime;
};

using PedalsSystemInstance = etl::singleton<PedalsSystem>;

#endif /* PEDALSSYSTEM */
