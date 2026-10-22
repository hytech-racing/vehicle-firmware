#include "test_drivebrain_controller.hpp"
#include "test_drivetrain.hpp"
#include "test_tcmux.hpp"
#include "test_vehicle_state_machine.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>


int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}
