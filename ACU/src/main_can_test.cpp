#include "CANInterface.hpp"

#include <Arduino.h>

const size_t CAN_BAUDRATE = 500000;

const uint32_t MSG_ID = 0x11;
const uint8_t MSG_DATA = 0x45;

const uint32_t DELAY = 10;

// Pointer, not a global: CANBus starts the hardware on construction, so it is built in setup()
CANBus<CAN3>* main_bus = nullptr;


void on_recv(const CAN_message_t& msg)
{
    Serial.println("msg recvd");
    Serial.print("MB: ");
    Serial.print(msg.mb);
    Serial.print("  ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print("  EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (auto b : msg.buf)
    {
        Serial.print(b);
        Serial.print(" ");
    }
    Serial.print("  TS: ");
    Serial.println(msg.timestamp);
}

void setup()
{
    static CANBus<CAN3> main_can(CAN_BAUDRATE);
    main_bus = &main_can;
}

void loop()
{
    delay(DELAY);
    CAN_message_t test_msg;
    test_msg.id = MSG_ID;
    test_msg.len = 1;
    test_msg.buf[0] = MSG_DATA;
    main_bus->enqueueOnBus(test_msg);
    main_bus->sendTxBuffer();

    main_bus->processRxBuffer(CANBus<CAN3>::RecvHandler_t::create<&on_recv>());
    // Serial.println("testing");
}
