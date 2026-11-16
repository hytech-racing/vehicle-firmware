#ifndef SENSOR_RANGE_CHECK_HPP
#define SENSOR_RANGE_CHECK_HPP

/**
 * @brief Checks whether a sensor reading is within inclusive limits.
 * @tparam T Numeric type shared by the reading and both limits.
 * @param sensor_value Sensor reading in the same units as the limits.
 * @param minimum_value Finite lower limit.
 * @param maximum_value Finite upper limit.
 * @return True if the reading is within the limits, false otherwise.
 */

template <typename T>
bool is_sensor_in_range(
    T sensor_value,
    T minimum_value,
    T maximum_value)
{
    return sensor_value >= minimum_value
        && sensor_value <= maximum_value;
}

#endif