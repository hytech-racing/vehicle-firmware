#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_interfaces/test_watchdog_interface.h"


int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}