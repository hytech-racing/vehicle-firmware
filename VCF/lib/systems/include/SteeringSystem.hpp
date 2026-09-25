#ifndef STEERING_SYSTEM_HPP
#define STEERING_SYSTEM_HPP

#include <cstdint>
#include <etl/singleton.h>
#include <cmath>

#include "SharedFirmwareTypes.h"



/**
 * @note OOR implausibility boundaries for one sensor (tolerance margin already applied).
 *       These are full boundary values, not margin offsets.
*/
struct SteeringSensorMargins_s
{
    int32_t min_with_margin_raw;
    int32_t max_with_margin_raw;
};

/**
 * @note Running min/max of raw sensor readings, continuously updated every tick
 *       Reset periodically if stale (see recalibrateSteering())
*/
struct SteeringSensorObservedExtremes_s
{
    uint32_t min_observed_raw_counts;
    uint32_t max_observed_raw_counts;
};

/**
 * @note Frozen snapshot of observed data for calibration, only updated when recalibrateSteering runs
 *       This is what conversion and implausibility checks actually read from
 */
struct SteeringSensorCalibration_s
{
    uint32_t min_signal_raw;  // Raw value at minimum (left) steering angle
    uint32_t max_signal_raw;  // Raw value at maximum (right) steering angle
    uint32_t span_raw;        // max - min, in raw counts
    uint32_t midpoint_raw;    // (max + min) / 2, in raw counts
};

struct SteeringSystemParams_s
{
    /* Per-sensor calibration (raw ADC bounds, span, midpoint) */
    SteeringSensorCalibration_s analog_calibration;
    SteeringSensorCalibration_s digital_calibration;

    /* Per-sensor OOR boundaries */
    SteeringSensorMargins_s analog_margins;
    SteeringSensorMargins_s digital_margins;

    SteeringSensorObservedExtremes_s analog_observed_extremes;
    SteeringSensorObservedExtremes_s digital_observed_extremes;

    /* Conversion Rates */
    float deg_per_count_analog;
    float deg_per_count_digital; // based on digital readings

    /* Implausibility Tolerances */
    float analog_tolerance_fraction;  // +- 0.5% error, applied against analog_calibration.span_raw
    float digital_tolerance_fraction; // +- 0.5% error, applied against digital_calibration.span_raw
    float analog_tolerance_margin_raw;  // derived: span_raw * analog_tolerance_fraction
    float digital_tolerance_margin_raw; // derived: span_raw * digital_tolerance_fraction

    float max_dtheta_threshold;            // Max change in angle since last reading to consider valid
    float error_between_sensors_tolerance; // Max allowed disagreement between sensors (degrees)
};

class SteeringSystem
{
public:

    SteeringSystem(const SteeringSystemParams_s &params) : _params(params) {}

    void recalibrateSteering();

    void evaluate_steering(const uint32_t analog_raw, const SteeringEncoderReading_s digital_data, const uint32_t current_millis);

    void update_observed_steering_limits(const uint32_t analog_raw, const uint32_t digital_raw);

    /* Getters */
    const SteeringSystemParams_s &get_steering_params() const { return _params; }
    const SteeringSystemData_s &get_steering_system_data() const { return _system_data; }
    float get_unfiltered_analog_steering_deg() const { return _analog_angle_unfiltered; }

    /* Setters */
    void set_steering_params(const SteeringSystemParams_s &params) { _params = params; }
    void set_steering_system_data(const SteeringSystemData_s &system_data) { _system_data = system_data; }

private:

    SteeringSystemData_s _system_data {};
    SteeringSystemParams_s _params;

    // Track the state of system at the previous tick to compare against current state for implausibility checks
    uint32_t _prev_timestamp = 0;

    float _analog_angle_unfiltered = 0.0f;
    float _prev_analog_angle = 0.0f;
    float _prev_digital_angle = 0.0f;
    float _prev_digital_vel_angle = 0.0f;
    float _prev_analog_vel_angle = 0.0f;
    bool _calibrating = false;
    bool _finished_calibrating = false;
    bool _first_run = true; // skip dTheta check on the very first tick

    /* 2nd-order Butterworth IIR low-pass on the analog angle. */
    // Designed for fc = 8 Hz at fs = 500 Hz. Direct Form II Transposed.
    float _bw_z1 = 0.0f;
    float _bw_z2 = 0.0f;
    bool  _bw_initialized = false;
    float _last_filtered_analog_angle = 0.0f;

    /* Coefficients */
    static constexpr float kBwB0 =  0.00235721f;
    static constexpr float kBwB1 =  0.00471442f;
    static constexpr float kBwB2 =  0.00235721f;
    static constexpr float kBwA1 = -1.85804330f;
    static constexpr float kBwA2 =  0.86747213f;


    /**
     *
    */
    float _convertDigitalSensor(const uint32_t digital_raw);

    float _convert_analog_sensor(const uint32_t analog_raw);

    float _filter_analog_angle(float x);

    /**
     * @brief
     * @return true if steering_analog is outside of the range defined by min and max sensor values, false otherwise
    */
    bool _evaluateAnalogSteeringOOR(const uint32_t steering_analog);

    /**
     * @brief
     * @return true if steering_digital is outside the range defined by min and max sensor values, false otherwise
    */
    bool _evaluateDigitalSteeringOOR(const uint32_t steering_digital);

    /**
     * @brief returns true if change in angle exceeds maximum change per reading ( max_dtheta_threshold )
     */
    bool _evaluate_steering_dtheta_exceeded(float dtheta);

};

using SteeringSystemInstance = etl::singleton<SteeringSystem>;


#endif