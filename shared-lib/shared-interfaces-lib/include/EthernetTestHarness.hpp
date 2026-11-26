#ifndef ETHERNET_TEST_HARNESS_HPP
#define ETHERNET_TEST_HARNESS_HPP

/**
 * @brief Barebones Ethernet bring-up test, shared by every Teensy 4.1 board's main_ethernet_test.cpp.
 *
 * Works like CANTestHarness: every board broadcasts a 1-byte test message every DELAY_MS on TEST_PORT,
 * and prints every message it receives. Connect two (or more) boards flashed with their ethernet test
 * and watch each board's serial monitor; the sender is identified by its IP.
 * Each board only picks its own IP (from EthernetAddressDefs.h).
 */

#include "EthernetAddressDefs.hpp"
#include <Arduino.h>
#include <QNEthernet.h>
#include <array>


class EthernetTestHarness
{
  public:
    static constexpr uint16_t TEST_PORT = 5555; // shared by every board, unused by the real firmware
    static constexpr uint8_t MSG_DATA = 0x45;
    static constexpr uint32_t DELAY_MS = 10;
    static constexpr size_t BUF_LEN = 8;

    static void setup(const IPAddress& ip)
    {
        qindesign::network::Ethernet.begin(ip, EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
        _socket.begin(TEST_PORT);
    }

    static void loop()
    {
        delay(DELAY_MS);

        // Print everything received since the last loop
        while (_socket.parsePacket() > 0)
        {
            _printPacket();
        }

        // Broadcast so any board on the network running the test receives it
        _socket.beginPacket(qindesign::network::Ethernet.broadcastIP(), TEST_PORT);
        _socket.write(MSG_DATA);
        _socket.endPacket();
    }

  private:
    static inline qindesign::network::EthernetUDP _socket;

    static void _printPacket()
    {
        std::array<uint8_t, BUF_LEN> buffer = {};
        int len = _socket.read(buffer.data(), buffer.size());

        Serial.println("msg recvd");
        Serial.print("FROM: ");
        Serial.print(_socket.remoteIP());
        Serial.print("  LEN: ");
        Serial.print(len);
        Serial.print(" DATA: ");
        for (int i = 0; i < len; i++)
        {
            Serial.print(buffer[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
};

#endif // ETHERNET_TEST_HARNESS_HPP
