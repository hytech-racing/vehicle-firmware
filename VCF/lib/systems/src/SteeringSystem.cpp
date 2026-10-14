#include "SteeringSystem.hpp"


void SteeringSystem::recalibrateSteering()
{
    _recalibrateSensor(_params.analog, ANALOG_RAW_LIMITS);
    _recalibrateSensor(_params.digital, DIGITAL_RAW_LIMITS);
}

void SteeringSystem::updateObservedExtremes(const uint32_t analog_raw, const uint32_t digital_raw)
{
    _updateSensorExtremes(_params.analog.observed_extremes, analog_raw, ANALOG_RAW_LIMITS);
    _updateSensorExtremes(_params.digital.observed_extremes, digital_raw, DIGITAL_RAW_LIMITS);
}

void SteeringSystem::evaluateSteering(const uint32_t analog_raw, const SteeringEncoderReading_s &digital_reading, const uint32_t current_micros)
{
    // Update implause and error data
    _system_data.analog_oor_implausibility          = false;
    _system_data.digital_oor_implausibility         = false;
    _system_data.sensor_disagreement_implausibility = false;
    _system_data.dtheta_exceeded_analog             = false;
    _system_data.dtheta_exceeded_digital            = false;
    _system_data.both_sensors_fail                  = false;

    // Update raw data
    const uint32_t digital_raw = digital_reading.rawValue;
    _system_data.analog_raw  = analog_raw;
    _system_data.digital_raw = digital_raw;
    _system_data.interface_sensor_error = (digital_reading.status == SteeringEncoderStatus_e::ERROR);

    // Update angle data
    _analog_angle_unfiltered_deg = _convertAnalogRawToDeg(analog_raw);
    const float digital_angle_deg = _convertDigitalRawToDeg(digital_raw);
    _system_data.digital_steering_angle = digital_angle_deg;

    if (_is_first_tick)
    {
        _analog_angle_filtered_deg = _applyAnalogLowpass(_analog_angle_unfiltered_deg);
        _system_data.analog_steering_angle = _analog_angle_filtered_deg;

        _last_internal_sample_us = current_micros;
        _digital_angle_at_last_internal_sample_deg = digital_angle_deg;

        _is_first_tick = false;
        return;
    }

    // Filter and rates update at ~500 Hz, between updates the last values are held
    const uint32_t elapsed_us = current_micros - _last_internal_sample_us;
    if (elapsed_us >= INTERNAL_SAMPLE_THRESHOLD_US)
    {
        const float new_filtered_analog_deg = _applyAnalogLowpass(_analog_angle_unfiltered_deg);

        const float elapsed_sec = static_cast<float>(elapsed_us) / US_PER_SECOND;
        _system_data.analog_steering_velocity_deg_s  = (new_filtered_analog_deg - _analog_angle_filtered_deg) / elapsed_sec;
        _system_data.digital_steering_velocity_deg_s = (digital_angle_deg - _digital_angle_at_last_internal_sample_deg) / elapsed_sec;

        _analog_angle_filtered_deg = new_filtered_analog_deg;
        _digital_angle_at_last_internal_sample_deg = digital_angle_deg;
        _last_internal_sample_us = current_micros;
    }

    const float analog_angle_deg = _analog_angle_filtered_deg;
    _system_data.analog_steering_angle = analog_angle_deg;

    // Update plausability check values
    _system_data.dtheta_exceeded_analog  = _isRateExceeded(_system_data.analog_steering_velocity_deg_s);
    _system_data.dtheta_exceeded_digital = _isRateExceeded(_system_data.digital_steering_velocity_deg_s);
    _system_data.analog_oor_implausibility  = _isOutOfRange(analog_raw, _params.analog.oor_bounds);
    _system_data.digital_oor_implausibility = _isOutOfRange(digital_raw, _params.digital.oor_bounds);
    const float sensor_disagreement_deg = std::fabs(analog_angle_deg - digital_angle_deg);
    _system_data.sensor_disagreement_implausibility = (sensor_disagreement_deg > _params.max_sensor_disagreement_deg);

    const bool is_analog_valid = !_system_data.analog_oor_implausibility
                              && !_system_data.dtheta_exceeded_analog;

    const bool is_digital_valid = !_system_data.digital_oor_implausibility
                               && !_system_data.dtheta_exceeded_digital
                               && !_system_data.interface_sensor_error;

    // Sensor selection: prefer digital (higher resolution), fall back to analog
    /// TODO: sensor_disagreement_implausibility is flagged but does not yet influence which sensor is trusted.
    if (is_digital_valid)
    {
        _system_data.output_steering_angle = digital_angle_deg;
    }
    else if (is_analog_valid)
    {
        _system_data.output_steering_angle = analog_angle_deg;
    }
    else
    {
        // Leave output_steering_angle unchanged, holding the last good value
        _system_data.both_sensors_fail = true;
    }
}

void SteeringSystem::_updateSensorExtremes(SteeringSensorObservedExtremes_s &extremes, const uint32_t raw, const SteeringSensorRawLimits_s &limits)
{
    if (raw < limits.min_unclipped_raw || raw > limits.max_unclipped_raw)
    {
        return;  // clipped reading, would pin the extremes to the rail
    }
    extremes.min_raw = std::min(extremes.min_raw, raw);
    extremes.max_raw = std::max(extremes.max_raw, raw);
}

void SteeringSystem::_recalibrateSensor(SteeringSensorParams_s &sensor, const SteeringSensorRawLimits_s &limits)
{
    SteeringSensorObservedExtremes_s &observed_extremes = sensor.observed_extremes;

    /**
     * @note If min > max, that just means no valid samples since the last reset
     *       Keep the previous calibration
    */
    if (observed_extremes.min_raw > observed_extremes.max_raw)
    {
        return;
    }

    SteeringSensorCalibration_s &calibration_snapshot = sensor.calibration_snapshot;
    calibration_snapshot.min_raw = observed_extremes.min_raw;
    calibration_snapshot.max_raw = observed_extremes.max_raw;
    calibration_snapshot.span_raw = calibration_snapshot.max_raw - calibration_snapshot.min_raw;
    calibration_snapshot.center_raw = calibration_snapshot.min_raw + (calibration_snapshot.span_raw / 2);

    sensor.oor_tolerance_raw = static_cast<float>(calibration_snapshot.span_raw) * sensor.oor_tolerance_fraction;
    sensor.oor_bounds.min_valid_raw = static_cast<int32_t>(static_cast<float>(calibration_snapshot.min_raw) - sensor.oor_tolerance_raw);
    sensor.oor_bounds.max_valid_raw = static_cast<int32_t>(static_cast<float>(calibration_snapshot.max_raw) + sensor.oor_tolerance_raw);

    // A span wider than the sensor can physically produce means the extremes are stale. Clear them so the next sweep starts fresh
    if (calibration_snapshot.span_raw > limits.max_plausible_span_raw)
    {
        observed_extremes = SteeringSensorObservedExtremes_s {};
    }
}

float SteeringSystem::_convertAnalogRawToDeg(const uint32_t analog_raw) const
{
    const int32_t counts_from_center = static_cast<int32_t>(analog_raw) - static_cast<int32_t>(_params.analog.calibration_snapshot.center_raw);
    return static_cast<float>(counts_from_center) * _params.analog.deg_per_count;
}

float SteeringSystem::_convertDigitalRawToDeg(const uint32_t digital_raw) const
{
    /**
     * @note The digital encoder is mounted reversed: raw counts decrease as steering angle increases,
     *       so the offset is taken the other way round to match the analog sign convention
    */
    const int32_t counts_from_center = static_cast<int32_t>(_params.digital.calibration_snapshot.center_raw) - static_cast<int32_t>(digital_raw);
    return static_cast<float>(counts_from_center) * _params.digital.deg_per_count;
}

float SteeringSystem::_applyAnalogLowpass(const float analog_angle_deg)
{
    /**
     * @note On the very first sample, we will pre-load the state to the steady-state values for this input
     *       so the output starts at the input instead of ramping up from 0 over ~50 ms.
    */
    if (!_is_lowpass_initialized)
    {
        _lowpass_state_1 = (1.0f - LOWPASS_B0) * analog_angle_deg;
        _lowpass_state_2 = (LOWPASS_B2 - LOWPASS_A2) * analog_angle_deg;
        _is_lowpass_initialized = true;
    }

    // Output = the new input's (small) share + everything history has already contributed.
    const float filtered_deg = (LOWPASS_B0 * analog_angle_deg) + _lowpass_state_1;

    // Prepare for the next call. Next tick, this tick's input/output become "1 step ago", so add their b1/a1 contributions,
    // plus the "2 steps ago" terms waiting in _lowpass_state_2
    _lowpass_state_1 = (LOWPASS_B1 * analog_angle_deg) - (LOWPASS_A1 * filtered_deg) + _lowpass_state_2;

    // Store this tick's contribution to the output two ticks from now (its b2/a2 terms) Next call, this gets folded into _lowpass_state_1 above
    _lowpass_state_2 = (LOWPASS_B2 * analog_angle_deg) - (LOWPASS_A2 * filtered_deg);

    return filtered_deg;
}

bool SteeringSystem::_isOutOfRange(const uint32_t raw, const SteeringSensorOutOfRangeBounds_s &bounds)
{
    const int32_t raw_signed = static_cast<int32_t>(raw);
    return (raw_signed < bounds.min_valid_raw) || (raw_signed > bounds.max_valid_raw);
}

bool SteeringSystem::_isRateExceeded(const float rate_deg_per_s) const
{
    return std::fabs(rate_deg_per_s) > _params.max_steering_rate_deg_per_s;
}