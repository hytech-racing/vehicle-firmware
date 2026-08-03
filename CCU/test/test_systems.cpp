#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_systems/test_state_machine.h"


int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}