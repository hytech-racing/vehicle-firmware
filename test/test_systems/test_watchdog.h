#define TEST_WATCHDOG
#include "gtest/gtest.h"
#include "WatchdogSystem.h"

#include "VCR_Constants.h"

TEST(WatchdogSystemTesting, test_watchdog) {
    WatchdogInstance::create(10);

    WatchdogSystem &watchdog = WatchdogInstance::instance();
    const int time_var = 0; // starting time
    const int time_arb = 2500; // arbitrary number greater than 2000

    const int end_time = 13;

    for(int time = 0; time < end_time; time++)
    {
        if(time <= 10)
        {   
            ASSERT_EQ(watchdog.get_watchdog_state(time), false);
        } else {
            ASSERT_EQ(watchdog.get_watchdog_state(time), true);
        }
    }
    
    ASSERT_EQ(watchdog.get_watchdog_state(time_arb), false);
    
    //Test an arbituary time above 1000 millisecs from previous time
    ASSERT_EQ(watchdog.get_watchdog_state(time_arb + 1000), true);
}
