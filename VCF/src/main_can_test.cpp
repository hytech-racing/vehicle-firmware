#include "VCF_Constants.hpp"
#include "CANTestHarness.h"

/**
 * VCF buses:
 *   CAN1 = TELEM  (VCFConstants::TELEM_CAN_BAUDRATE)
 *   CAN2 = FAUX   (VCFConstants::FAUX_CAN_BAUDRATE)
 */
using TestHarness = CANTestHarness<CAN1>;
const uint32_t CAN_BAUDRATE = VCFConstants::TELEM_CAN_BAUDRATE;
const uint32_t MSG_ID = 0x13;

void setup()
{
    TestHarness::setup(CAN_BAUDRATE, MSG_ID);
}

void loop()
{
    TestHarness::loop();
}
