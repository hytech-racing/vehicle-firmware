#include "ACU_Constants.hpp"
#include "CANTestHarness.h"

/**
 * ACU buses:
 *   CAN2 = CCU    (ACUConstants::VEH_CAN_BAUDRATE)
 *   CAN3 = EM     (ACUConstants::EM_CAN_BAUDRATE)
 */
using TestHarness = CANTestHarness<CAN3>;
const uint32_t CAN_BAUDRATE = ACUConstants::EM_CAN_BAUDRATE;
const uint32_t MSG_ID = 0x11;

void setup()
{
    TestHarness::setup(CAN_BAUDRATE, MSG_ID);
}

void loop()
{
    TestHarness::loop();
}
