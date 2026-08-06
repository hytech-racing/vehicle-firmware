#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_buzzer.h"
#include "test_pedals_system.h"
#include "test_steering_system.h"

int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}