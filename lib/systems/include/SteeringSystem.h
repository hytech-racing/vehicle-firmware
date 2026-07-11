#ifndef STEERING_SYSTEM_H
#define STEERING_SYSTEM_H

/* Standard Library */
#include <cstdint>

/* ETL Library */
#include <etl/singleton.h>

/* External Includes */
#include <cmath>
#include "SharedFirmwareTypes.h"


struct SteeringParams_s
{
    /* Raw ADC Input Signals */
    uint32_t min_steering_signal_analog;  //Raw ADC value from analog sensor at minimum (left) steering angle (calibration)
    uint32_t max_steering_signal_analog;  //Raw ADC value from analog sensor at maximum (right) steering angle
    uint32_t min_steering_signal_digital; //Raw ADC value from digital sensor at minimum (left) steering angle
    uint32_t max_steering_signal_digital; //Raw ADC value from digital sensor at maximum (right) steering angle

    /* Boundaries with tolerance margins applied */
    int32_t analog_min_with_margins;  // Added margins to min raw value
    int32_t analog_max_with_margins;  // Added margins to max raw value
    int32_t digital_min_with_margins; // Added margins to min raw value
    int32_t digital_max_with_margins; // Added margins to max raw value

    /* Derived span and midpoint values */
    uint32_t span_signal_analog;  // range of the analog sensor in counts
    uint32_t span_signal_digital; // range of the digital sensor in counts
    uint32_t digital_midpoint;    // midpoint of raw values
    uint32_t analog_midpoint;     // midpoint of raw values

    /* Conversion Rates */
    // float deg_per_count_analog = 0.0439f; // hard coded for analog sensor that is 180
    float deg_per_count_analog;
    float deg_per_count_digital; //based on digital readings

    /* Implausibility Tolerances */
    float analog_tolerance;  //+- 0.5% error
    float digital_tolerance; // +- 0.2 degrees error
    float analog_tol_deg;
    float digital_tol_deg;   // +- 0.2 degrees error
    float max_dtheta_threshold; // Maximum change in angle since last reading to consider the reading valid
    float error_between_sensors_tolerance; // Max allowed disagreement between sensors (degrees)
};

class SteeringSystem
{
public:

    SteeringSystem(const SteeringParams_s &params) : _params(params) {}

    void recalibrate_steering_digital();

    void evaluate_steering(const uint32_t analog_raw, const SteeringEncoderReading_s digital_data, const uint32_t current_millis);

    void update_observed_steering_limits(const uint32_t analog_raw, const uint32_t digital_raw);

    /* Getters */
    const SteeringParams_s &get_steering_params() const { return _params; }
    const SteeringSystemData_s &get_steering_system_data() const { return _system_data; }
    float get_unfiltered_analog_steering_deg() const { return _analog_angle_unfiltered; }

    /* Setters */
    void set_steering_params(const SteeringParams_s &params) { _params = params; }
    void set_steering_system_data(const SteeringSystemData_s &system_data) { _system_data = system_data; }

private:

    SteeringSystemData_s _system_data {};
    SteeringParams_s _params;

    // Track the state of system at the previous tick to compare against current state for implausibility checks
    uint32_t _prev_timestamp = 0;
    uint32_t _min_observed_analog = UINT32_MAX;
    uint32_t _max_observed_analog = 0;
    uint32_t _min_observed_digital = UINT32_MAX;
    uint32_t _max_observed_digital = 0;
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

    float _convert_digital_sensor(const uint32_t digital_raw);

    float _convert_analog_sensor(const uint32_t analog_raw);

    float _filter_analog_angle(float x);

    /**
     * @brief returns true if steering_analog is outside of the range defined by min and max sensor values
     */
    bool _evaluate_steering_oor_analog(const uint32_t steering_analog);

    /**
     * @brief returns true if steering_digital is outside the range defined by min and max sensor values
     */
    bool _evaluate_steering_oor_digital(const uint32_t steering_digital);

    /**
     * @brief returns true if change in angle exceeds maximum change per reading ( max_dtheta_threshold )
     */
    bool _evaluate_steering_dtheta_exceeded(float dtheta);

};

using SteeringSystemInstance = etl::singleton<SteeringSystem>;


#endif

