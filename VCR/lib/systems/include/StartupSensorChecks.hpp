#ifndef VCR_STARTUP_SENSOR_CHECKS_HPP
#define VCR_STARTUP_SENSOR_CHECKS_HPP

namespace VCRStartupSensorChecks
{
    // @brief Checks the rear left load cell reading.
    // @param rear_left_load_cell_reading Converted sensor reading.
    // @param rear_left_load_cell_minimum Startup lower limit in the same units.
    // @param rear_left_load_cell_maximum Startup upper limit in the same units.
    // @return True if the reading is within the limits, including both endpoints, false otherwise.
    bool check_rear_left_load_cell(
        float rear_left_load_cell_reading,
        float rear_left_load_cell_minimum,
        float rear_left_load_cell_maximum);

    // @brief Checks the rear right load cell reading.
    // @param rear_right_load_cell_reading Converted sensor reading.
    // @param rear_right_load_cell_minimum Startup lower limit in the same units.
    // @param rear_right_load_cell_maximum Startup upper limit in the same units.
    // @return True if the reading is within the limits, including both endpoints, false otherwise.
    bool check_rear_right_load_cell(
        float rear_right_load_cell_reading,
        float rear_right_load_cell_minimum,
        float rear_right_load_cell_maximum);

    // @brief Checks the rear left suspension potentiometer reading.
    // @param rear_left_suspension_pot_reading Converted sensor reading.
    // @param rear_left_suspension_pot_minimum Startup lower limit in the same units.
    // @param rear_left_suspension_pot_maximum Startup upper limit in the same units.
    // @return True if the reading is within the limits, including both endpoints, false otherwise.
    bool check_rear_left_suspension_pot(
        float rear_left_suspension_pot_reading,
        float rear_left_suspension_pot_minimum,
        float rear_left_suspension_pot_maximum);

    // @brief Checks the rear right suspension potentiometer reading.
    // @param rear_right_suspension_pot_reading Converted sensor reading.
    // @param rear_right_suspension_pot_minimum Startup lower limit in the same units.
    // @param rear_right_suspension_pot_maximum Startup upper limit in the same units.
    // @return True if the reading is within the limits, including both endpoints, false otherwise.
    bool check_rear_right_suspension_pot(
        float rear_right_suspension_pot_reading,
        float rear_right_suspension_pot_minimum,
        float rear_right_suspension_pot_maximum);
}

#endif