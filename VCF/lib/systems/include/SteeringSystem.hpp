#ifndef STEERING_SYSTEM_HPP
#define STEERING_SYSTEM_HPP

#include <cstdint>
#include <algorithm>
#include <cmath>
#include <etl/singleton.h>
#include "SharedFirmwareTypes.h"

/**
 * @note Struct holds the running min/max of raw readings seen since the last reset.
 *       If min_raw > max_raw, this just means "no valid samples yet"
*/
struct SteeringSensorObservedExtremes_s
{
    uint32_t min_raw = UINT32_MAX;
    uint32_t max_raw = 0;
};

/**
 * @note Struct is a snapshot of the observed extremes, taken only when recalibrateSteering() runs.
 *       Angle conversion reads from this, never from the live extremes
*/
struct SteeringSensorCalibration_s
{
    uint32_t min_raw;     // smallest raw count seen during the sweep
    uint32_t max_raw;     // largest raw count seen during the sweep
    uint32_t span_raw;    // max_raw - min_raw
    uint32_t center_raw;  // raw count treated as 0 degrees (straight ahead)
};

/**
 * @note Struct holds the out-of-range limits: calibrated min/max widened by the tolerance on each side.
 *       A raw reading outside [min_valid_raw, max_valid_raw] is flagged implausible
*/
struct SteeringSensorOutOfRangeBounds_s
{
    int32_t min_valid_raw;
    int32_t max_valid_raw;
};

/**
 * @note Struct holds the fixed raw-count sanity limits for one sensor type
*/
struct SteeringSensorRawLimits_s
{
    uint32_t min_unclipped_raw;       // readings below this are treated as clipped / stuck low
    uint32_t max_unclipped_raw;       // readings above this are treated as clipped / stuck high
    uint32_t max_plausible_span_raw;  // an observed span wider than this means the extremes are stale
};

/**
 * @note An instance of this struct represents a single sensor
*/
struct SteeringSensorParams_s
{
    /* Configuration */
    float deg_per_count;           // degrees of steering per raw count
    float oor_tolerance_fraction;  // fraction of span_raw added to each side of the OOR bounds, e.g. 0.005 = 0.5%

    /* Live state, updated every tick */
    SteeringSensorObservedExtremes_s observed_extremes {};

    /* Derived by recalibrateSteering() */
    SteeringSensorCalibration_s calibration_snapshot {};
    float oor_tolerance_raw = 0.0f;  // span_raw * oor_tolerance_fraction
    SteeringSensorOutOfRangeBounds_s oor_bounds {};
};

struct SteeringSystemParams_s
{
    SteeringSensorParams_s analog;
    SteeringSensorParams_s digital;

    float max_steering_rate_deg_per_s;  // a sensor reporting a faster rate than this is implausible
    float max_sensor_disagreement_deg;  // max |analog - digital| before flagging disagreement
};

class SteeringSystem
{
public:

    explicit SteeringSystem(const SteeringSystemParams_s &params) : _params(params) {};

    /**
     * @brief Wrapper, calling _recalibrateSensor() for both the digial and analog sensors
    */
    void recalibrateSteering();

    /**
     * @brief Wrapper, calling _updateSensorExtremes() for both the digial and analog sensors
    */
    void updateObservedExtremes(uint32_t analog_raw, uint32_t digital_raw);

    /** Converts, filters, plausibility-checks and selects the output steering angle for one tick. */
    void evaluateSteering(uint32_t analog_raw, const SteeringEncoderReading_s &digital_reading, uint32_t current_millis);

    /* Getters */
    const SteeringSystemParams_s &getSteeringParams() const { return _params; }
    const SteeringSystemData_s &getSteeringSystemData() const { return _system_data; }
    float getUnfilteredAnalogSteeringDeg() const { return _analog_angle_unfiltered_deg; }

    /* Setters */
    void setSteeringParams(const SteeringSystemParams_s &params) { _params = params; }
    void setSteeringSystemData(const SteeringSystemData_s &system_data) { _system_data = system_data; }

private:

    SteeringSystemParams_s _params;
    SteeringSystemData_s _system_data {};
    bool _is_first_tick = true;  // no history yet, so rate checks are skipped

    /**
     * @note Filter and steering-rate calculations run every other call of the 1 kHz steering task because
     *       the low-pass coefficients are designed for 500 Hz; changing the rate requires regenerating them
     * @note We gate at 1500 us (1.5 task periods) instead of 2000 us so scheduler jitter can't skip a sample
     */
    static constexpr uint32_t INTERNAL_SAMPLE_THRESHOLD_US = 1500;
    static constexpr float US_PER_SECOND = 1.0e6f;

    uint32_t _last_internal_sample_us = 0;
    float _digital_angle_at_last_internal_sample_deg = 0.0f;  // reference angle for digital velocity

    float _analog_angle_unfiltered_deg = 0.0f;
    float _analog_angle_filtered_deg = 0.0f;  // filter output, held between updates; also the reference for analog velocity

    // Analog: 360 deg sensor through the VCF resistor divider. Typical full-lock span is ~2000 counts.
    // Digital: 14-bit encoder (0..16383) with a 10-count guard band at each end. Typical full-lock span is ~9000 counts.
    static constexpr SteeringSensorRawLimits_s ANALOG_RAW_LIMITS {5, 3675, 2500};
    static constexpr SteeringSensorRawLimits_s DIGITAL_RAW_LIMITS {10, (1U << 14U) - 10, 9000};

    /**
     * @brief Using a 2nd-order Butterworth low-pass
     * @note fc = 8 Hz at fs = 500 Hz
     * @note H(z) = ( B0 + B1/z + B2/(z^2) ) / ( 1+ A1/z = A2/(z^2))
    */
    static constexpr float LOWPASS_B0 =  0.00235721f;
    static constexpr float LOWPASS_B1 =  0.00471442f;
    static constexpr float LOWPASS_B2 =  0.00235721f;
    static constexpr float LOWPASS_A1 = -1.85804330f;
    static constexpr float LOWPASS_A2 =  0.86747213f;
    float _lowpass_state_1 = 0.0f; // equivalent to b1·x[n-1] − a1·y[n-1] + b2·x[n-2] − a2·y[n-2]
    float _lowpass_state_2 = 0.0f; // equivalent to b2·x[n-1] − a2·y[n-1]
    bool _is_lowpass_initialized = false;

    /**
     * @brief Takes one raw reading and updates a sensor's running min/max (works for digital and analog)
     * @note Readings outside [limits.min_unclipped_raw, limits.max_unclipped_raw] are treated as clipped and ignored entirely
     * @note Accepted readings can only widen the range; it is narrowed only when _recalibrateSensor() clears the extremes after an implausible span
    */
    static void _updateSensorExtremes(SteeringSensorObservedExtremes_s &extremes, uint32_t raw, const SteeringSensorRawLimits_s &limits);

    /**
     * @brief Recalibrates one sensor (analog or digital) from its observed extremes
     * @note Copies observed_extremes into calibration_snapshot, then recomputes oor_tolerance_raw and oor_bounds from the new span
     * @note If no valid samples have been observed, nothing is changed and the previous calibration is kept
     * @note If the new span exceeds limits.max_plausible_span_raw, observed_extremes is cleared so the next sweep starts fresh
     *       (the calibration from this pass is still applied)
    */
    static void _recalibrateSensor(SteeringSensorParams_s &sensor, const SteeringSensorRawLimits_s &limits);

    float _convertAnalogRawToDeg(uint32_t analog_raw) const;

    float _convertDigitalRawToDeg(uint32_t digital_raw) const;

   /**
     * @brief Smooths the analog steering angle with a 2nd-order Butterworth low-pass filter
     * @note The filter blends each new reading with its own recent history, so slow, real steering
     *       motion passes through while fast noise (ADC jitter, electrical noise) is averaged out
     * @note Cutoff is 8 Hz at a 500 Hz update rate. Must be called once per rate-update tick (every 2 ms)
     *       calling it at a different rate shifts the cutoff frequency
     * @note The math being computed is: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
     *       where x is the raw angle and y is the filtered angle
     * @param analog_angle_deg New unfiltered analog angle (x[n])
     * @return Filtered analog angle (y[n])
    */
    float _applyAnalogLowpass(float analog_angle_deg);

    /**
     * @return True if a raw reading is outside [min_valid_raw, max_valid_raw], false otherwise
    */
    static bool _isOutOfRange(uint32_t raw, const SteeringSensorOutOfRangeBounds_s &bounds);

    /**
     * @return True if the rate of angle change > max_steering_rate_deg_per_s, false otherwise
    */
    bool _isRateExceeded(float rate_deg_per_s) const;

};

using SteeringSystemInstance = etl::singleton<SteeringSystem>;

#endif