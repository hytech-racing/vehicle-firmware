/**
 * Front IR temp sensor programmer (env: vcf_ir_temp_program)
 *
 * Reprograms one Izze IRTS-V3 sensor on VCF FRONT_AUX CAN (CAN2): CAN base ID, emissivity, sampling
 * frequency, channel count, bit rate (datasheet Rev.F "Sensor Configuration").
 *
 * 1. Plug in ONE sensor (every sensor listening on CURRENT_BASE_CAN_ID takes the new config).
 * 2. Set the values below, flash, and open the serial monitor at 115200.
 * 3. It sends the programming message at 1 Hz for 11 s, then prints DONE and stops.
 * 4. Power cycle the sensor to apply. Repeat for the other sensor.
 *
 * Front sensors: FL = 0x4C4, FR = 0x4C8, 500 kbit/s (FRONT_AUX). Firmware decodes these IDs.
 * After programming, the sensor runs at 500 kbit/s, so set CURRENT_BITRATE = 500000 to reprogram it again.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace
{
    /* ---- What the sensor is set to right now ---- */
    constexpr uint32_t CURRENT_BITRATE = 1'000'000;   // factory default 1 Mbit/s
    constexpr uint16_t CURRENT_BASE_CAN_ID = 0x4B0; // the sensor listens for programming on its current base ID. Factory: LF 0x4B0, RF 0x4B4

    /* ---- What to program it to ---- */
    constexpr uint16_t NEW_BASE_CAN_ID = 0x4C4;   // FL = 0x4C4, FR = 0x4C8
    constexpr uint8_t NEW_EMISSIVITY = 85;        // 1 = 0.01 .. 100 = 1.00 (factory default 85). TODO: tune for brake rotors
    constexpr uint8_t NEW_SAMPLING_FREQ_CODE = 6; // 1 = 1Hz, 2 = 2Hz, 3 = 4Hz, 4 = 8Hz, 5 = 16Hz, 6 = 32Hz, 7 = 64Hz (60deg FOV only), 8 = 100Hz
    constexpr uint8_t NEW_CHANNELS = 160;         // 40 = 4 ch, 80 = 8 ch, 160 = 16 ch
    constexpr uint8_t NEW_BITRATE_CODE = 2;       // 1 = 1M, 2 = 500k, 3 = 250k, 4 = 100k. FRONT_AUX runs at 500k

    /* Datasheet "Sensor Configuration" */
    constexpr uint16_t PROGRAMMING_CONSTANT = 0x7530; // 30000
    constexpr unsigned long SEND_PERIOD_MS = 1000;
    constexpr unsigned long SEND_DURATION_MS = 11000; // datasheet minimum is 10 s

    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> front_aux_can;
    CAN_message_t config_msg;
    unsigned long start_millis = 0;
    unsigned long last_send_millis = 0;
    bool isConfigChangeDone = false;
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    front_aux_can.begin();
    front_aux_can.setBaudRate(CURRENT_BITRATE);

    config_msg.id = CURRENT_BASE_CAN_ID;
    config_msg.len = 8;
    config_msg.buf[0] = PROGRAMMING_CONSTANT >> 8; // big-endian
    config_msg.buf[1] = PROGRAMMING_CONSTANT & 0xFF;
    config_msg.buf[2] = NEW_BASE_CAN_ID >> 8;
    config_msg.buf[3] = NEW_BASE_CAN_ID & 0xFF;
    config_msg.buf[4] = NEW_EMISSIVITY;
    config_msg.buf[5] = NEW_SAMPLING_FREQ_CODE;
    config_msg.buf[6] = NEW_CHANNELS;
    config_msg.buf[7] = NEW_BITRATE_CODE;

    Serial.printf(
        "IR temp programmer: sending to 0x%03X at %lu bps for %lu s:",
        CURRENT_BASE_CAN_ID,
        static_cast<unsigned long>(CURRENT_BITRATE),
        SEND_DURATION_MS / 1000);
    for (uint8_t i = 0; i < 8; i++)
    {
        Serial.printf(" %02X", config_msg.buf[i]);
    }
    Serial.println();

    start_millis = millis();
    last_send_millis = start_millis - SEND_PERIOD_MS; // send right away
}

void loop()
{
    if (isConfigChangeDone)
    {
        return;
    }

    const unsigned long now = millis();
    if ((now - start_millis) > SEND_DURATION_MS)
    {
        // Datasheet: do NOT continuously send messages on the sensor's base ID
        isConfigChangeDone = true;
        Serial.println("DONE. Power cycle the sensor to apply the new config.");
        return;
    }

    if ((now - last_send_millis) >= SEND_PERIOD_MS)
    {
        front_aux_can.write(config_msg);
        last_send_millis = now;
        Serial.print(".");
    }
}
