#ifndef SENSOR_RANGE_CHECK_HPP
#define SENSOR_RANGE_CHECK_HPP

// @brief Checks whether a sensor reading is within inclusive limits.
// @tparam T Numeric type shared by the reading and both limits.
// @param sensor_value Sensor reading.
// @param minimum_value Finite lower limit in the same units.
// @param maximum_value Finite upper limit in the same units.
// @return True if the reading is within the limits, including both endpoints, false otherwise.
template <typename T>
bool sensor_range_check(
    T sensor_value,
    T minimum_value,
    T maximum_value)
{
    return sensor_value >= minimum_value 
    && sensor_value <= maximum_value;
}

#endif