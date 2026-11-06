#include "CCU_Constants.hpp"
#include "CANTestHarness.h"

/**
 * CCU buses:
 *   CAN1 = ACU      (CCUConstants::ACU_CAN_BAUDRATE)
 *   CAN3 = CHARGER  (CCUConstants::CHARGER_CAN_BAUDRATE)
 */
using TestHarness = CANTestHarness<CAN1>;
const uint32_t CAN_BAUDRATE = CCUConstants::ACU_CAN_BAUDRATE;
const uint32_t MSG_ID = 0x12;

void setup()
{
    TestHarness::setup(CAN_BAUDRATE, MSG_ID);
}

void loop()
{
    TestHarness::loop();
}
