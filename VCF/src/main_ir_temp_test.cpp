/**
 * Front IR temp sensor programmer (env: vcf_ir_temp_test)
 *
 * Reprograms one Izze IRTS-V3 sensor on VCF FRONT_AUX CAN (CAN2): CAN base ID, emissivity, sampling
 * frequency, channel count, bit rate (datasheet Rev.F "Sensor Configuration").
 *
 * 1. Flash and open the serial monitor at 115200.
 * 2. SCAN: it first listens (listen-only, never transmits) at each sensor bit rate and prints every CAN ID it hears.
 *    Each sensor sends 4 consecutive IDs starting at its base ID.
 * 3. PROGRAM (PROGRAM_SENSOR = true): it then sends the programming message on CURRENT_BASE_CAN_ID at CURRENT_BITRATE at 1 Hz for 15 s,
 *    prints DONE and stops. Only a sensor whose base ID is CURRENT_BASE_CAN_ID takes the new config, so both
 *    sensors can stay on the bus as long as their base IDs differ.
 * 4. Power cycle the sensor to apply. Set the values for the other sensor and repeat.
 *
 * If the scan shows the sensor on a different ID / rate than CURRENT_*, fix CURRENT_* and flash again.
 * Sending to the wrong ID / rate does nothing.
 *
 * Front sensors: FL = 0x4C4, FR = 0x4C8, 1 Mbit/s (FRONT_AUX). Firmware decodes these IDs.
 * Both fronts are the standard IRTS-V3 (4 IDs each), so FL's block ends at 0x4C7 and doesn't touch FR.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace
{
    /* ---- Step 1: scan only (false). Step 2: set CURRENT_* from the scan, set true to program ---- */
    constexpr bool PROGRAM_SENSOR = false;

    /* ---- What the sensor is set to right now (check the SCAN output) ---- */
    // Factory: 1 Mbit/s, base IDs LF 0x4B0, RF 0x4B4, LR 0x4B8, RR 0x4BC
    constexpr uint32_t CURRENT_BITRATE = 1000000;
    constexpr uint16_t CURRENT_BASE_CAN_ID = 0x4B4; // the sensor only listens for programming on its current base ID

    /* ---- What to program it to ---- */
    constexpr uint16_t NEW_BASE_CAN_ID = 0x4C8;   // FL = 0x4C4, FR = 0x4C8
    constexpr uint8_t NEW_EMISSIVITY = 100;       // 1 = 0.01 .. 100 = 1.00 (factory default 85)
    constexpr uint8_t NEW_SAMPLING_FREQ_CODE = 6; // 1 = 1Hz, 2 = 2Hz, 3 = 4Hz, 4 = 8Hz, 5 = 16Hz, 6 = 32Hz, 7 = 64Hz (60deg FOV only), 8 = 100Hz
    constexpr uint8_t NEW_CHANNELS = 160;         // 40 = 4 ch, 80 = 8 ch, 160 = 16 ch
    constexpr uint8_t NEW_BITRATE_CODE = 1;       // 1 = 1M, 2 = 500k, 3 = 250k, 4 = 100k. FRONT_AUX runs at 1M

    /* Datasheet "Sensor Configuration" */
    constexpr uint16_t PROGRAMMING_CONSTANT = 0x7530; // 30000
    constexpr unsigned long SEND_PERIOD_MS = 1000;
    constexpr unsigned long SEND_DURATION_MS = 15000; // datasheet minimum is 10 s, with margin

    /* Scan */
    constexpr uint32_t SCAN_BITRATES[] = {1000000, 500000, 250000, 100000}; // every rate the sensor supports
    constexpr unsigned long SCAN_DURATION_PER_RATE_MS = 5000;
    constexpr size_t MAX_SCAN_IDS = 32;

    FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> front_aux_can;
    CAN_message_t config_msg;
    unsigned long start_millis = 0;
    unsigned long last_send_millis = 0;
    bool isConfigChangeDone = false;

    /**
     * @brief Listens (without ever transmitting) at each bit rate and prints the standard CAN IDs heard
     */
    void scanBus()
    {
        Serial.println("SCAN (listen-only):");
        for (const uint32_t bitrate : SCAN_BITRATES)
        {
            front_aux_can.setBaudRate(bitrate, LISTEN_ONLY);

            uint32_t ids[MAX_SCAN_IDS] = {};
            uint32_t counts[MAX_SCAN_IDS] = {};
            size_t num_ids = 0;
            const unsigned long scan_start = millis();
            while ((millis() - scan_start) < SCAN_DURATION_PER_RATE_MS)
            {
                CAN_message_t msg;
                if (!front_aux_can.read(msg) || msg.flags.extended)
                {
                    continue;
                }
                size_t index = 0;
                while ((index < num_ids) && (ids[index] != msg.id))
                {
                    index++;
                }
                if (index == num_ids)
                {
                    if (num_ids == MAX_SCAN_IDS)
                    {
                        continue;
                    }
                    ids[num_ids++] = msg.id;
                }
                counts[index]++;
            }

            Serial.printf("  %7lu bps:", static_cast<unsigned long>(bitrate));
            if (num_ids == 0)
            {
                Serial.println(" nothing");
                continue;
            }
            Serial.println();
            for (uint32_t id = 0; id <= 0x7FF; id++) // print in ID order
            {
                for (size_t i = 0; i < num_ids; i++)
                {
                    if (ids[i] == id)
                    {
                        Serial.printf(
                            "    0x%03lX  %4lu frames  (%lu Hz)\n",
                            static_cast<unsigned long>(id),
                            static_cast<unsigned long>(counts[i]),
                            static_cast<unsigned long>(counts[i] * 1000 / SCAN_DURATION_PER_RATE_MS));
                    }
                }
            }
        }
    }
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    front_aux_can.begin();
    scanBus();
    if (!PROGRAM_SENSOR)
    {
        Serial.println("Scan only (PROGRAM_SENSOR = false). Nothing was sent.");
        isConfigChangeDone = true;
        return;
    }
    front_aux_can.setBaudRate(CURRENT_BITRATE); // back to normal (transmitting) mode for programming

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

    Serial
        .printf("PROGRAM: sending to 0x%03X at %lu bps for %lu s:", CURRENT_BASE_CAN_ID, static_cast<unsigned long>(CURRENT_BITRATE), SEND_DURATION_MS / 1000);
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
        const bool is_queued = front_aux_can.write(config_msg) > 0;
        last_send_millis = now;
        Serial.print(is_queued ? "." : "x"); // x = TX mailbox full, frame not sent
    }
}
