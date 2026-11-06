#include "RDC_Constants.hpp"
#include "CANTestHarness.h"

/**
 * RDC buses:
 *   CAN1 = DRIVERLESS  (RDCConstants::DRIVERLESS_CAN_BAUDRATE)
 */
using TestHarness = CANTestHarness<CAN1>;
const uint32_t CAN_BAUDRATE = RDCConstants::DRIVERLESS_CAN_BAUDRATE;
const uint32_t MSG_ID = 0x15;

void setup()
{
    TestHarness::setup(CAN_BAUDRATE, MSG_ID);
}

void loop()
{
    TestHarness::loop();
}
