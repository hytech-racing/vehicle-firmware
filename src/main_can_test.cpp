#include <Arduino.h>

#include "CANInterface.h"

FlexCAN_T4<CAN1> MAIN_CAN;

const uint32_t CAN_BAUDRATE = 500000;

const uint32_t MSG_ID = 0x15;
const uint8_t MSG_DATA = 0x45;

const uint32_t DELAY = 10;

void on_recv(const CAN_message_t &msg)
{
    Serial.print("recieved");
    Serial.println("msg recvd");
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    for ( uint8_t i = 0; i < 8; i++ ) {
      Serial.print(msg.buf[i]); Serial.print(" ");
    }
    Serial.print("  TS: "); Serial.println(msg.timestamp);
}

void setup()
{
    handle_CAN_setup(MAIN_CAN, CAN_BAUDRATE, &on_recv);
}

void loop()
{
    delay(DELAY);
    CAN_message_t test_msg;
    test_msg.id = MSG_ID;
    test_msg.len = 1;
    test_msg.buf[0] = MSG_DATA;
    MAIN_CAN.write(test_msg);
    // Serial.println("testing");
}