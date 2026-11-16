#ifndef TEST_SENSOR_RANGE_CHECK_HPP
#define TEST_SENSOR_RANGE_CHECK_HPP

#include <gtest/gtest.h>
#include "StartupSensorChecks.hpp"

TEST(StartupSensorChecks, ChecksAcceleratorChannelOne)
{
    using namespace VCFStartupSensorChecks;

    // Accelerator channel one.
    EXPECT_FALSE(check_accelerator_channel_one(9u, 10u, 20u));
    EXPECT_TRUE(check_accelerator_channel_one(10u, 10u, 20u));
    EXPECT_TRUE(check_accelerator_channel_one(15u, 10u, 20u));
    EXPECT_TRUE(check_accelerator_channel_one(20u, 10u, 20u));
    EXPECT_FALSE(check_accelerator_channel_one(21u, 10u, 20u));
}

#endif