#include "gtest/gtest.h"
#include "BuzzerController.h"

int base = 2500; // arbitrary number greater than 2000

class BuzzerControllerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        BuzzerControllerInstance::create();
        // Reset state between tests, since the singleton persists
        // across the whole binary, not just one TEST_F.
        BuzzerControllerInstance::instance().deactivate();
    }
};

TEST_F(BuzzerControllerTest, initial_state)
{
    auto &buzzer = BuzzerControllerInstance::instance();
    ASSERT_EQ(buzzer.buzzer_is_active(base), false);
}

TEST_F(BuzzerControllerTest, activate_buzzer)
{
    auto &buzzer = BuzzerControllerInstance::instance();
    buzzer.activate(base);
    ASSERT_EQ(buzzer.buzzer_is_active(base+10), true);
    ASSERT_EQ(buzzer.buzzer_is_active(base+2010), false);
}

TEST_F(BuzzerControllerTest, interrupt_buzzer)
{
    auto &buzzer = BuzzerControllerInstance::instance();
    buzzer.activate(base);
    ASSERT_EQ(buzzer.buzzer_is_active(base+10), true);
    buzzer.deactivate();
    ASSERT_EQ(buzzer.buzzer_is_active(base+20), false);
}