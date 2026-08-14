#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_acu_controller.h"
#include "test_acu_state_machine.h"
#include "test_soc_kalman.h"


int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}