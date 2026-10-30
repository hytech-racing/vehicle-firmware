#include <gtest/gtest.h>

#include "test_drivetrain.hpp"
#include "test_tcmux.hpp"
#include "test_vehicle_state_machine.hpp"

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
