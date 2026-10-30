#include "test_drivebrain_controller.hpp"
#include "test_drivetrain.hpp"
#include "test_tcmux.hpp"
#include "test_vehicle_state_machine.hpp"

#include <gtest/gtest.h>

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
