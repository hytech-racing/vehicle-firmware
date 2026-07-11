#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "pedals_system_test.h"
#include "test_buzzer.h"
#include "steering_system_test.h"

int main(int argc, char **argv)
{

    testing::InitGoogleMock(&argc, argv);
	if (RUN_ALL_TESTS())
	;
	// Always return zero-code and allow PlatformIO to parse results
	return 0;
}
