#include "CANTestHarness.h"
#include "RDC_Constants.hpp"
#include "core_pins.h"

/**
 * RDC buses:
 *   CAN1 = DRIVERLESS  (RDCConstants::DRIVERLESS_CAN_BAUDRATE)
 */
// using TestHarness = CANTestHarness<CAN1>;
// const uint32_t CAN_BAUDRATE = RDCConstants::DRIVERLESS_CAN_BAUDRATE;
// const uint32_t MSG_ID = 0x15;

unsigned long long last_millis = 0;

void setup()
{
    // TestHarness::setup(CAN_BAUDRATE, MSG_ID);
    Serial.begin(115200);
    analogReadResolution(12);

    pinMode(A4, INPUT);
    pinMode(A3, INPUT);
    pinMode(A2, INPUT);
    pinMode(A1, INPUT);
    pinMode(A0, INPUT);
}

void loop()
{
    // TestHarness::loop();
    if (millis() - last_millis > 100)
    {
        last_millis = millis();
        Serial.println();
        Serial.println(analogRead(A4));
        Serial.println(analogRead(A3));
        Serial.println(analogRead(A2));
        Serial.println(analogRead(A1));
        Serial.println(analogRead(A0));
    }
}
