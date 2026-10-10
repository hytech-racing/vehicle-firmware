#include "test_driverless_state_machine.hpp"
#include "test_driverless_system.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
