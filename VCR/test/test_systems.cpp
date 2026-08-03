#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_systems/test_drivebrain_controller.h"
#include "test_systems/test_drivetrain.h"
#include "test_systems/test_tcmux.h"
#include "test_systems/test_vehicle_state_machine.h"


int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}