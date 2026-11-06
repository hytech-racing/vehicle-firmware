#include "VCR_Constants.hpp"
#include "CANTestHarness.h"

/**
 * VCR buses:
 *   CAN1 = TELEM     (VCRConstants::TELEM_CAN_BAUDRATE)
 *   CAN2 = RAUX      (VCRConstants::RAUX_CAN_BAUDRATE)
 *   CAN3 = INVERTER  (VCRConstants::INVERTER_CAN_BAUDRATE)
 */
using TestHarness = CANTestHarness<CAN1>;
const uint32_t CAN_BAUDRATE = VCRConstants::TELEM_CAN_BAUDRATE;
const uint32_t MSG_ID = 0x14;

void setup()
{
    TestHarness::setup(CAN_BAUDRATE, MSG_ID);
}

void loop()
{
    TestHarness::loop();
}
