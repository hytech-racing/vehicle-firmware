#ifndef TEST_STARTUP_SENSOR_CHECKS_HPP
#define TEST_STARTUP_SENSOR_CHECKS_HPP

#include <gtest/gtest.h>
#include "StartupSensorChecks.hpp"

TEST(StartupSensorChecks, ChecksEachRearSensor)
{
    using namespace VCRStartupSensorChecks;

    // Rear left load cell.
    EXPECT_FALSE(check_rear_left_load_cell(9.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_left_load_cell(10.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_left_load_cell(15.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_left_load_cell(20.0f, 10.0f, 20.0f));
    EXPECT_FALSE(check_rear_left_load_cell(21.0f, 10.0f, 20.0f));

    // Rear right load cell.
    EXPECT_FALSE(check_rear_right_load_cell(9.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_right_load_cell(10.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_right_load_cell(15.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_right_load_cell(20.0f, 10.0f, 20.0f));
    EXPECT_FALSE(check_rear_right_load_cell(21.0f, 10.0f, 20.0f));

    // Rear left suspension potentiometer.
    EXPECT_FALSE(check_rear_left_suspension_pot(9.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_left_suspension_pot(10.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_left_suspension_pot(15.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_left_suspension_pot(20.0f, 10.0f, 20.0f));
    EXPECT_FALSE(check_rear_left_suspension_pot(21.0f, 10.0f, 20.0f));

    // Rear right suspension potentiometer.
    EXPECT_FALSE(check_rear_right_suspension_pot(9.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_right_suspension_pot(10.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_right_suspension_pot(15.0f, 10.0f, 20.0f));
    EXPECT_TRUE(check_rear_right_suspension_pot(20.0f, 10.0f, 20.0f));
    EXPECT_FALSE(check_rear_right_suspension_pot(21.0f, 10.0f, 20.0f));
}

#endif