#include "CANInterface.hpp"

// Pointers, not globals: CANBus starts the hardware on construction, so it is built in setup()
CANBus<CAN1>* telem_bus = nullptr;
CANBus<CAN2>* faux_bus = nullptr;

const uint32_t CAN_BAUDRATE = 1000000;

const uint32_t MSG_ID = 0x11;
const uint8_t MSG_DATA = 0x45;

const uint32_t DELAY = 10;

void on_recv_telem(const CAN_message_t& msg)
{
    Serial.println("msg recvd on telem can!");
    Serial.print("MB: ");
    Serial.print(msg.mb);
    Serial.print("  ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print("  EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (uint8_t byte : msg.buf)
    {
        Serial.print(byte);
        Serial.print(" ");
    }
    Serial.print("  TS: ");
    Serial.println(msg.timestamp);
}

void on_recv_faux(const CAN_message_t& msg)
{
    Serial.println("msg recvd on faux can!");
    Serial.print("MB: ");
    Serial.print(msg.mb);
    Serial.print("  ID: 0x");
    Serial.print(msg.id, HEX);
    Serial.print("  EXT: ");
    Serial.print(msg.flags.extended);
    Serial.print("  LEN: ");
    Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (uint8_t byte : msg.buf)
    {
        Serial.print(byte);
        Serial.print(" ");
    }
    Serial.print("  TS: ");
    Serial.println(msg.timestamp);
}

void setup()
{
    static CANBus<CAN1> telem(CAN_BAUDRATE);
    static CANBus<CAN2> faux(CAN_BAUDRATE);
    telem_bus = &telem;
    faux_bus = &faux;
}

void loop()
{
    delay(DELAY);
    CAN_message_t test_msg;
    test_msg.id = MSG_ID;
    test_msg.len = 1;
    test_msg.buf[0] = MSG_DATA;
    telem_bus->enqueue(test_msg);
    faux_bus->enqueue(test_msg);
    telem_bus->sendTxBuffer();
    faux_bus->sendTxBuffer();

    telem_bus->processRxBuffer(CANBus<CAN1>::RecvHandler_t::create<&on_recv_telem>());
    faux_bus->processRxBuffer(CANBus<CAN2>::RecvHandler_t::create<&on_recv_faux>());
    // Serial.println("testing");
}
