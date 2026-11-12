#include "test_buzzer_controller.hpp"
#include "test_neopixel_controller.hpp"
#include "test_pedals_system.hpp"
#include "test_steering_system.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

int main(int argc, char** argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
