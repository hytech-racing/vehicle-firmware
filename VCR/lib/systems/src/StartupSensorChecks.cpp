#include "StartupSensorChecks.hpp"
#include "SensorRangeCheck.hpp"

namespace VCRStartupSensorChecks
{
    bool check_rear_left_load_cell(
        float rear_left_load_cell_reading,
        float rear_left_load_cell_minimum,
        float rear_left_load_cell_maximum)
    {
        return sensor_range_check(
            rear_left_load_cell_reading, 
            rear_left_load_cell_minimum, 
            rear_left_load_cell_maximum);
    }

    bool check_rear_right_load_cell(
        float rear_right_load_cell_reading,
        float rear_right_load_cell_minimum,
        float rear_right_load_cell_maximum)
    {
        return sensor_range_check(
            rear_right_load_cell_reading,
            rear_right_load_cell_minimum,
            rear_right_load_cell_maximum);
    }

    bool check_rear_left_suspension_pot(
        float rear_left_suspension_pot_reading,
        float rear_left_suspension_pot_minimum,
        float rear_left_suspension_pot_maximum)
    {
        return sensor_range_check(
            rear_left_suspension_pot_reading,
            rear_left_suspension_pot_minimum,
            rear_left_suspension_pot_maximum);
    }

    bool check_rear_right_suspension_pot(
        float rear_right_suspension_pot_reading,
        float rear_right_suspension_pot_minimum,
        float rear_right_suspension_pot_maximum)
    {
        return sensor_range_check(
            rear_right_suspension_pot_reading,
            rear_right_suspension_pot_minimum,
            rear_right_suspension_pot_maximum);
    }
}