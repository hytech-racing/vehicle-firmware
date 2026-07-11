#include "gtest/gtest.h"
#include "BuzzerController.h"

BuzzerController &buzzer = BuzzerController::getInstance();
int base = 2500; // arbitrary number greater than 2000

TEST (BuzzerControllerTesting, initial_state) {
    ASSERT_EQ(buzzer.buzzer_is_active(base), false);
}

TEST (BuzzerControllerTesting, activate_buzzer) {
    buzzer.activate(base);
    ASSERT_EQ(buzzer.buzzer_is_active(base+10), true);
    ASSERT_EQ(buzzer.buzzer_is_active(base+2010), false);
}

TEST (BuzzerControllerTesting, interrupt_buzzer) {
    buzzer.activate(base);
    ASSERT_EQ(buzzer.buzzer_is_active(base+10), true);
    buzzer.deactivate();
    ASSERT_EQ(buzzer.buzzer_is_active(base+20), false);
}