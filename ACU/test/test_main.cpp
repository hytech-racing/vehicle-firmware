#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_acu_controller.hpp"
#include "test_acu_state_machine.hpp"
#include "test_soc_kalman.hpp"


int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}