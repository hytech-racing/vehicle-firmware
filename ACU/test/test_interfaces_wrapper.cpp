// Thin wrapper so PlatformIO's test discovery finds an entry point
// directly in test/, while the real test code and its headers stay
// together in test_systems/ where automatic library discovery works.
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_interfaces/main_interfaces_unit_tests.cpp"

int main(int argc, char **argv)
{
    testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}