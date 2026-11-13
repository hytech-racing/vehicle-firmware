/**
 * TTPMS receiver programmer (env: vcr_ttpms_test)
 *
 * Reprograms the Izze W-REC-V2 receiver on VCR REAR_AUX CAN (CAN2): CAN base ID, bit rate, emissivity mode
 * (datasheet Appendix A.1).
 *
 * 1. Set the values below, flash, and open the serial monitor at 115200.
 * 2. It sends the programming message at 1 Hz for 11 s, then prints DONE and stops.
 * 3. Unplug the receiver's power for 5+ s and plug it back in to apply.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace
{
    /* ---- What the receiver is set to right now ---- */
    constexpr uint32_t CURRENT_BITRATE = 1000000;  // bus rate the receiver is running at
    constexpr uint16_t CURRENT_BASE_CAN_ID = 0x424; // the receiver listens for programming on its current base ID

    /* ---- What to program it to ---- */
    constexpr uint16_t NEW_BASE_CAN_ID = 0x424;    // FL = base, FR = +6, RL = +12, RR = +18 (firmware decodes 0x424)
    constexpr uint8_t NEW_BITRATE_CODE = 1;        // 1 = 1M, 2 = 500k, 3 = 250k, 4 = 100k
    constexpr uint8_t NEW_EMISSIVITY_MODE = 1;     // 0 = standard, 1 = default cal, 2 = custom

    /* Datasheet A.1 */
    constexpr uint16_t BASE_CONFIG_COMMAND_ID = 0x7530; // "Programming Constant"
    constexpr uint8_t SENSOR_ASSIGNMENT = 1;            // only documented value
    constexpr unsigned long SEND_PERIOD_MS = 1000;
    constexpr unsigned long SEND_DURATION_MS = 11000;   // datasheet minimum is 10 s

    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> rear_aux_can;
    CAN_message_t config_msg;
    unsigned long start_millis = 0;
    unsigned long last_send_millis = 0;
    bool isConfigChangeDone = false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    rear_aux_can.begin();
    rear_aux_can.setBaudRate(CURRENT_BITRATE);

    config_msg.id = CURRENT_BASE_CAN_ID;
    config_msg.len = 8;
    config_msg.buf[0] = BASE_CONFIG_COMMAND_ID >> 8; // big-endian
    config_msg.buf[1] = BASE_CONFIG_COMMAND_ID & 0xFF;
    config_msg.buf[2] = NEW_BASE_CAN_ID >> 8;
    config_msg.buf[3] = NEW_BASE_CAN_ID & 0xFF;
    config_msg.buf[4] = SENSOR_ASSIGNMENT;
    config_msg.buf[5] = NEW_BITRATE_CODE;
    config_msg.buf[6] = NEW_EMISSIVITY_MODE;
    config_msg.buf[7] = 0;

    Serial.printf("TTPMS programmer: sending to 0x%03X at %lu bps for %lu s:", CURRENT_BASE_CAN_ID,
                  static_cast<unsigned long>(CURRENT_BITRATE), SEND_DURATION_MS / 1000);
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
        // Datasheet: do NOT keep sending to the receiver
        isConfigChangeDone = true;
        Serial.println("DONE. Unplug the receiver's power for 5+ s, then plug it back in.");
        return;
    }

    if ((now - last_send_millis) >= SEND_PERIOD_MS)
    {
        rear_aux_can.write(config_msg);
        last_send_millis = now;
        Serial.print(".");
    }
}
