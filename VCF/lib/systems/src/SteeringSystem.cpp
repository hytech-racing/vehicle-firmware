#include "SteeringSystem.h"

void SteeringSystem::recalibrate_steering_digital()
{
    _params.min_steering_signal_analog = _min_observed_analog;
    _params.max_steering_signal_analog = _max_observed_analog;
    _params.min_steering_signal_digital = _min_observed_digital;
    _params.max_steering_signal_digital = _max_observed_digital;

    // Swap  min & max in the params if sensor is flipped
    if (_params.min_steering_signal_digital > _params.max_steering_signal_digital)
    {
        std::swap(_params.min_steering_signal_digital, _params.max_steering_signal_digital);
    }
    if (_params.min_steering_signal_analog > _params.max_steering_signal_analog)
    {
        std::swap(_params.min_steering_signal_analog, _params.max_steering_signal_analog);
    }

    _params.span_signal_analog  = _params.max_steering_signal_analog - _params.min_steering_signal_analog;
    _params.span_signal_digital = _params.max_steering_signal_digital-_params.min_steering_signal_digital;

    _params.analog_tol_deg  = static_cast<float>(_params.span_signal_analog) * _params.analog_tolerance * _params.deg_per_count_analog;
    _params.digital_tol_deg = static_cast<float>(_params.span_signal_digital) *_params.digital_tolerance * _params.deg_per_count_digital;

    _params.analog_midpoint  = (_params.max_steering_signal_analog + _params.min_steering_signal_analog) / 2;
    _params.digital_midpoint = (_params.max_steering_signal_digital + _params.min_steering_signal_digital) / 2;

    _params.analog_min_with_margins  = static_cast<int32_t>(_params.min_steering_signal_analog - _params.analog_tol_deg); // NOLINT
    _params.analog_max_with_margins  = static_cast<int32_t>(_params.max_steering_signal_analog + _params.analog_tol_deg); // NOLINT
    _params.digital_min_with_margins = static_cast<int32_t>(_params.min_steering_signal_digital - _params.digital_tol_deg); // NOLINT
    _params.digital_max_with_margins = static_cast<int32_t>(_params.max_steering_signal_digital + _params.digital_tol_deg); // NOLINT

    if (_max_observed_analog > _min_observed_analog && _params.span_signal_analog > 2500) // NOLINT with 360 deg analog sensor, typical span is about 2000
    {
        _min_observed_analog = UINT32_MAX; // after calculating params, if the range is marginally greater than half the steering wheel adc, likely the min and max are clinging to a prior run that is not applicable, meaning we will need to reset the boundaries.
        _max_observed_analog = 0;
    }
    if (_max_observed_digital > _min_observed_digital && _params.span_signal_digital > 9000) // NOLINT with digital sensor, typical span is about 9000
    {
        _min_observed_digital = UINT32_MAX;
        _max_observed_digital = 0;
    }
}

void SteeringSystem::evaluate_steering(const uint32_t analog_raw, const SteeringEncoderReading_s digital_data, const uint32_t current_millis)
{
    // Reset flags
    _system_data.digital_oor_implausibility = false;
    _system_data.analog_oor_implausibility = false;
    _system_data.sensor_disagreement_implausibility = false;
    _system_data.dtheta_exceeded_analog = false;
    _system_data.dtheta_exceeded_digital = false;
    _system_data.both_sensors_fail = false;

    const uint32_t digital_raw = digital_data.rawValue;
    _system_data.interface_sensor_error = (digital_data.status == SteeringEncoderStatus_e::ERROR);
    _system_data.digital_raw = digital_raw;

    _system_data.analog_raw = analog_raw;
    _analog_angle_unfiltered = _convert_analog_sensor(analog_raw);

    // Conversion from raw ADC to degrees
    _system_data.digital_steering_angle = _convert_digital_sensor(digital_raw);

    uint32_t dt = 0;
    if (current_millis - _prev_timestamp >= 2)
    {
        dt = current_millis - _prev_timestamp; //current_millis is seperate data input  
    }

    if (!_first_run) // Check that we not on the first run which would mean no previous data
    {
        if (dt >= 2)
        {
            float filtered_analog_angle = _filter_analog_angle(_analog_angle_unfiltered);
            _system_data.analog_steering_angle = filtered_analog_angle; // Update the angle to the filtered value for downstream use and velocity calculation
            float dtheta_analog = filtered_analog_angle - _prev_analog_vel_angle;
            float dtheta_digital = _system_data.digital_steering_angle - _prev_digital_vel_angle;

            _system_data.analog_steering_velocity_deg_s = (dtheta_analog / static_cast<float>(dt)) * 1000.0f; // NOLINT 1000.0f is result of converting dt in millis to seconds
            _system_data.digital_steering_velocity_deg_s = (dtheta_digital / static_cast<float>(dt)) * 1000.0f; // NOLINT 1000.0f is result of converting dt in millis to seconds

            _last_filtered_analog_angle = filtered_analog_angle;
        }
        else
        {
            _system_data.analog_steering_angle = _last_filtered_analog_angle;
        }

        // Check if either sensor moved too much in one tick
        _system_data.dtheta_exceeded_analog = _evaluate_steering_dtheta_exceeded(_system_data.analog_steering_velocity_deg_s);
        _system_data.dtheta_exceeded_digital = _evaluate_steering_dtheta_exceeded(_system_data.digital_steering_velocity_deg_s); // use digital velocity for dtheta check since it's more precise and we are concerned about large changes in angle that could be caused by noise in the analog sensor

        // Check if either sensor is out of range (pass in raw)
        _system_data.analog_oor_implausibility = _evaluate_steering_oor_analog(analog_raw);
        _system_data.digital_oor_implausibility = _evaluate_steering_oor_digital(digital_raw);

        // Check if there is too much of a difference between sensor values
        float sensor_difference = std::fabs(_system_data.analog_steering_angle - _system_data.digital_steering_angle);
        bool sensors_agree = (sensor_difference <= _params.error_between_sensors_tolerance); //steeringParams.error
        _system_data.sensor_disagreement_implausibility = !sensors_agree;

        // Create an algorithm/checklist to determine which sensor we trust more,
        // or, if we should have an algorithm to have a weighted calculation based on both values
        bool analog_valid = !_system_data.analog_oor_implausibility && !_system_data.dtheta_exceeded_analog;
        bool digital_valid = !_system_data.digital_oor_implausibility && !_system_data.dtheta_exceeded_digital && !_system_data.interface_sensor_error;

        if (analog_valid && digital_valid)
        {
            /// NOTE: This is currently not doing anything essentially. It doesn't matter whether or not sensors agree.
            // If sensors have acceptable difference, use digital as steering angle
            if (sensors_agree)
            {
                _system_data.output_steering_angle = _system_data.digital_steering_angle;
            }
            else
            {
                _system_data.output_steering_angle = _system_data.digital_steering_angle; // Default to original, but we need to consider what we really want to put here
            }
        }
        else if (analog_valid)
        {
            _system_data.output_steering_angle = _system_data.analog_steering_angle;
        }
        else if (digital_valid)
        {
            _system_data.output_steering_angle = _system_data.digital_steering_angle;
        } else // if both sensors fail
        {
            _system_data.output_steering_angle = _prev_digital_angle;
            _system_data.both_sensors_fail = true;
        }
    }

    // Update states, 500Hz
    if (dt >= 2)
    {
        _prev_timestamp = current_millis;
        _prev_analog_vel_angle = _system_data.analog_steering_angle;
        _prev_digital_vel_angle = _system_data.digital_steering_angle;
    }

    _prev_analog_angle = _system_data.analog_steering_angle;
    _prev_digital_angle = _system_data.digital_steering_angle;
    _first_run = false;
}

void SteeringSystem::update_observed_steering_limits(const uint32_t analog_raw, const uint32_t digital_raw)
{
    _min_observed_analog = std::min(_min_observed_analog, analog_raw);
    _max_observed_analog = std::max(_max_observed_analog, analog_raw);
    _min_observed_digital = std::min(_min_observed_digital, digital_raw); //NOLINT should both be uint32_t
    _max_observed_digital = std::max(_max_observed_digital, digital_raw); //NOLINT ^

    if (_min_observed_analog < 5) // NOLINT want to prevent sticking at 0 or clipping with small value
    {
        _min_observed_analog = UINT32_MAX; // clipping if it is at 0, it is likely sensor is clipping or clipped in past and reading is holding the 0 value.
    }
    if (_max_observed_analog > 3675) // NOLINT prevents clipping, this is slightly less than calculated value of actual max output of sensor with current resistor divider on VCF's ADC
    {
        _max_observed_analog = 0; // clipping
    }
    if (_min_observed_digital < 10) // NOLINT want to prevent sticking at 0 or clipping
    {
        _min_observed_digital = UINT32_MAX; // clipping on prior run.
    }
    if (_max_observed_digital > 16374) // NOLINT 16374 = 2^14 - 10 to prevent clipping with 14 bit resolution on sensor
    {
        _max_observed_digital = 0; // clipping
    }
}

float SteeringSystem::_convert_digital_sensor(const uint32_t digital_raw)
{
    const int32_t offset =  _params.digital_midpoint-static_cast<int32_t>(digital_raw); //NOLINT
    return static_cast<float>(offset) * _params.deg_per_count_digital; // bc diital sensor is flipped
}

float SteeringSystem::_convert_analog_sensor(const uint32_t analog_raw)
{
    // Get the raw value
    const int32_t offset = static_cast<int32_t>(analog_raw) - _params.analog_midpoint; //NOLINT
    return static_cast<float>(offset) * _params.deg_per_count_analog;
}

bool SteeringSystem::_evaluate_steering_oor_analog(const uint32_t steering_analog_raw) // RAW
{
    return (static_cast<int32_t>(steering_analog_raw) < _params.analog_min_with_margins || static_cast<int32_t>(steering_analog_raw) > _params.analog_max_with_margins);
}

bool SteeringSystem::_evaluate_steering_oor_digital(const uint32_t steering_digital_raw) // RAW
{
    return (static_cast<int32_t>(steering_digital_raw) < _params.digital_min_with_margins || static_cast<int32_t>(steering_digital_raw) > _params.digital_max_with_margins);
}

bool SteeringSystem::_evaluate_steering_dtheta_exceeded(float steering_velocity_deg_s)
{
    return (std::fabs(steering_velocity_deg_s) > _params.max_dtheta_threshold);
}

float SteeringSystem::_filter_analog_angle(float x)
{
    // First sample: pre-load the state so the output starts at x and
    // there is no startup transient (otherwise the filter would ramp
    // from 0 up to the first real value over ~50 ms).
    if (!_bw_initialized)
    {
        _bw_z1 = (1.0f - kBwB0) * x;
        _bw_z2 = (kBwB2 - kBwA2) * x;
        _bw_initialized = true;
    }

    // Direct Form II Transposed biquad: 5 multiplies, 4 adds, 2 floats of state.
    float y = kBwB0 * x + _bw_z1;
    _bw_z1 = kBwB1 * x - kBwA1 * y + _bw_z2;
    _bw_z2 = kBwB2 * x - kBwA2 * y;
    return y;
}
