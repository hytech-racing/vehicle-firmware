#include <Arduino.h>

#include "CANInterface.h"

FlexCAN_T4<CAN1> TELEM_CAN;
FlexCAN_T4<CAN2> FAUX_CAN;

const uint32_t CAN_BAUDRATE = 1000000;

const uint32_t MSG_ID = 0x11;
const uint8_t MSG_DATA = 0x45;

const uint32_t DELAY = 10;

void on_recv_telem(const CAN_message_t &msg)
{
    Serial.println("msg recvd on telem can!");
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (uint8_t byte : msg.buf) {
      Serial.print(byte); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);
}
    
void on_recv_faux(const CAN_message_t &msg)
{
    Serial.println("msg recvd on faux can!");
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for (uint8_t byte : msg.buf) {
      Serial.print(byte); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);
}

void setup()
{
    
    handle_CAN_setup(TELEM_CAN, CAN_BAUDRATE, &on_recv_telem);
    handle_CAN_setup(FAUX_CAN, CAN_BAUDRATE, &on_recv_faux);
}

void loop()
{
    delay(DELAY);
    CAN_message_t test_msg;
    test_msg.id = MSG_ID;
    test_msg.len = 1;
    test_msg.buf[0] = MSG_DATA;
    TELEM_CAN.write(test_msg);
    FAUX_CAN.write(test_msg);
    // Serial.println("testing");
}