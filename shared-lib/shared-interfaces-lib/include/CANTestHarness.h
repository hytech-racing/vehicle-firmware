#ifndef CAN_TEST_HARNESS_H
#define CAN_TEST_HARNESS_H

/**
 * @brief Barebones CAN bring-up test, shared by every Teensy board's main_can_test.cpp.
 *
 * Sends a 1-byte test message every DELAY_MS and prints every message received.
 * Each board only picks the peripheral (CAN1/CAN2/CAN3), baudrate, and TX ID.
 */

#include "CANInterface.h"
#include <Arduino.h>


template <CAN_DEV_TABLE CAN_DEV>
class CANTestHarness
{
  public:
    static constexpr uint8_t MSG_DATA = 0x45;
    static constexpr uint32_t DELAY_MS = 10;

    static void setup(uint32_t baudrate, uint32_t tx_id)
    {
        _tx_id = tx_id;
        handle_CAN_setup(_bus, baudrate, &_onRecv);
    }

    static void loop()
    {
        delay(DELAY_MS);
        CAN_message_t test_msg;
        test_msg.id = _tx_id;
        test_msg.len = 1;
        test_msg.buf[0] = MSG_DATA;
        _bus.write(test_msg);
    }

  private:
    static inline FlexCAN_T4<CAN_DEV> _bus;
    static inline uint32_t _tx_id = 0;

    static void _onRecv(const CAN_message_t& msg)
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
        for (uint8_t byte : msg.buf)
        {
            Serial.print(byte);
            Serial.print(" ");
        }
        Serial.print("  TS: ");
        Serial.println(msg.timestamp);
    }
};

#endif // CAN_TEST_HARNESS_H
