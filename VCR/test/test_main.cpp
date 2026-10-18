#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_drivebrain_controller.hpp"
#include "test_drivetrain.hpp"
#include "test_tcmux.hpp"
#include "test_vehicle_state_machine.hpp"


int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}