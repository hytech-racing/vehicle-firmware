#ifndef TEST_SENSOR_RANGE_CHECK_HPP
#define TEST_SENSOR_RANGE_CHECK_HPP

#include <gtest/gtest.h>

#include "SensorRangeCheck.hpp"

TEST(SensorRangeCheck, ChecksInclusiveLimits)
{
    EXPECT_FALSE(sensor_range_check(9, 10, 20));
    EXPECT_TRUE(sensor_range_check(10, 10, 20));
    EXPECT_TRUE(sensor_range_check(15, 10, 20));
    EXPECT_TRUE(sensor_range_check(20, 10, 20));
    EXPECT_FALSE(sensor_range_check(21, 10, 20));
}

#endif
