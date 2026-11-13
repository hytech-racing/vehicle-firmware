/**
 * TTPMS bench test (env: vcr_ttpms_test)
 *
 * Izze HS-TTPMS-V2 receiver on VCR REAR_AUX CAN (CAN2). Lets you change the CAN IDs and read the data back.
 * Open the serial monitor at 115200 baud, line ending "Newline", and type `help`.
 *
 * ID layout: one 6-ID block per sensor, back to back from a single base ID:
 *   FL = base, FR = base + 6, RL = base + 12, RR = base + 18
 *
 * Changing the receiver's CAN base ID / bit rate (datasheet A.1):
 *   1. `rate 1000000`              talk to the receiver at its current bit rate (factory default 1 Mbit/s)
 *   2. `prog 0x424 0x424 2`        send to current ID 0x424: new base 0x424, bit rate code 2 (500 kbit/s)
 *   3. wait for "DONE" (~11 s), then unplug the receiver's power for 5 s and plug it back in
 *   4. the test switches its decode base / bus rate to the new values automatically when the sequence finishes
 */

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "CANInterface.h"
#include "VCRCANInterfaceImpl.hpp"
#include "TTPMSInterface.hpp"

namespace
{
    constexpr uint32_t SERIAL_BAUD = 115200;
    constexpr uint32_t DEFAULT_CAN_BAUDRATE = 1000000; // receiver factory default
    constexpr unsigned long PRINT_PERIOD_MS = 1000;
    constexpr unsigned long SENSOR_TIMEOUT_MS = 5000;
    constexpr size_t LINE_BUF_LEN = 96;
    constexpr size_t MAX_SEEN_IDS = 64;

    /* Receiver programming (datasheet Appendix A.1). Send at 1 Hz for at least 10 s, then power cycle the receiver for 5 s */
    constexpr uint16_t BASE_CONFIG_COMMAND_ID = 0x7530;  // "Programming Constant": CAN base ID, sensor assignment, bit rate, emissivity mode
    constexpr uint8_t SENSOR_ASSIGNMENT_DEFAULT = 1;     // only documented value
    constexpr uint8_t MAX_EMISSIVITY_MODE = 2;           // 0 = standard, 1 = default cal, 2 = custom
    constexpr uint16_t MAX_RECEIVER_BASE_CAN_ID = 0x7FF - (ttpms_default_params::MSGS_PER_SENSOR - 1); // one sensor block must fit in 11 bits
    constexpr unsigned long CONFIG_SEND_PERIOD_MS = 1000;
    constexpr unsigned long CONFIG_SEND_DURATION_MS = 11000; // 1 s margin over the 10 s minimum

    struct SeenID_s
    {
        uint32_t id;
        uint32_t count;
        bool extended;
        uint8_t len;
        uint8_t buf[8];
        unsigned long last_millis;
    };

    std::array<SeenID_s, MAX_SEEN_IDS> seen_ids = {};
    size_t num_seen_ids = 0;
    uint32_t seen_overflow_count = 0;

    char line_buf[LINE_BUF_LEN] = {};
    size_t line_len = 0;

    uint32_t can_baudrate = DEFAULT_CAN_BAUDRATE;
    bool print_raw = false;
    bool print_table = true;
    unsigned long last_print_millis = 0;

    /* Programming sequence state */
    CAN_message_t config_msg;
    bool config_active = false;
    unsigned long config_start_millis = 0;
    unsigned long last_config_send_millis = 0;

    /* Applied once a programming sequence finishes */
    uint16_t pending_base_id = 0;
    uint32_t pending_baudrate = 0;

    const char *const CORNER_NAMES[ttpms_default_params::NUM_SENSORS] = {"FL", "FR", "RL", "RR"};

    uint32_t baudrate_from_code(uint8_t code)
    {
        switch (code)
        {
            case 1: return 1000000;
            case 2: return 500000;
            case 3: return 250000;
            case 4: return 100000;
            default: return 0;
        }
    }

    bool is_valid_baudrate(uint32_t rate)
    {
        return rate == 1000000 || rate == 500000 || rate == 250000 || rate == 100000;
    }

    bool parse_uint(const char *tok, uint32_t &out)
    {
        if (tok == nullptr)
        {
            return false;
        }
        char *end = nullptr;
        out = strtoul(tok, &end, 0); // base 0: accepts 0x424 (hex) or 1060 (decimal)
        return end != tok && *end == '\0';
    }

    /**
     * Builds the receiver base config message (datasheet A.1).
     * target_can_id is the ID the receiver listens on for programming (its current base ID).
     * @return false if any argument is out of range
     */
    bool build_base_config_msg(uint32_t target_can_id, uint32_t new_base_id, uint32_t rate_code, uint32_t emissivity_mode, CAN_message_t &msg)
    {
        if (target_can_id > 0x7FF || new_base_id < ttpms_default_params::MIN_BASE_CAN_ID || new_base_id > MAX_RECEIVER_BASE_CAN_ID ||
            baudrate_from_code(static_cast<uint8_t>(rate_code)) == 0 || rate_code > 0xFF || emissivity_mode > MAX_EMISSIVITY_MODE)
        {
            return false;
        }

        msg = CAN_message_t();
        msg.id = target_can_id;
        msg.len = 8;
        msg.buf[0] = static_cast<uint8_t>(BASE_CONFIG_COMMAND_ID >> 8); // big-endian
        msg.buf[1] = static_cast<uint8_t>(BASE_CONFIG_COMMAND_ID & 0xFF);
        msg.buf[2] = static_cast<uint8_t>(new_base_id >> 8);
        msg.buf[3] = static_cast<uint8_t>(new_base_id & 0xFF);
        msg.buf[4] = SENSOR_ASSIGNMENT_DEFAULT;
        msg.buf[5] = static_cast<uint8_t>(rate_code);
        msg.buf[6] = static_cast<uint8_t>(emissivity_mode);
        msg.buf[7] = 0;
        return true;
    }

    TTPMSSingleSensorData_s sensor_at(const TTPMSAllSensorData_s &data, size_t i)
    {
        switch (i)
        {
            case 0: return data.fl_ttpms;
            case 1: return data.fr_ttpms;
            case 2: return data.rl_ttpms;
            default: return data.rr_ttpms;
        }
    }

    void set_can_baudrate(uint32_t rate)
    {
        can_baudrate = rate;
        VCRCANInterfaceInstance::instance().REAR_AUX_CAN.setBaudRate(rate);
        Serial.printf("REAR_AUX CAN bit rate -> %lu\n", static_cast<unsigned long>(rate));
    }

    void record_seen_id(const CAN_message_t &msg)
    {
        for (size_t i = 0; i < num_seen_ids; i++)
        {
            if (seen_ids[i].id == msg.id && seen_ids[i].extended == static_cast<bool>(msg.flags.extended))
            {
                seen_ids[i].count++;
                seen_ids[i].len = msg.len;
                memcpy(seen_ids[i].buf, msg.buf, 8);
                seen_ids[i].last_millis = millis();
                return;
            }
        }
        if (num_seen_ids >= MAX_SEEN_IDS)
        {
            seen_overflow_count++;
            return;
        }
        SeenID_s &entry = seen_ids[num_seen_ids++];
        entry.id = msg.id;
        entry.count = 1;
        entry.extended = msg.flags.extended;
        entry.len = msg.len;
        memcpy(entry.buf, msg.buf, 8);
        entry.last_millis = millis();
    }

    void print_raw_msg(const CAN_message_t &msg)
    {
        Serial.printf("RX 0x%03lX%s len %u :", static_cast<unsigned long>(msg.id), msg.flags.extended ? " (ext)" : "", msg.len);
        for (uint8_t i = 0; i < msg.len && i < 8; i++)
        {
            Serial.printf(" %02X", msg.buf[i]);
        }
        Serial.println();
    }

    void print_help()
    {
        Serial.println();
        Serial.println("TTPMS test commands (numbers: 0x prefix = hex, otherwise decimal)");
        Serial.println("  rate <bps>                          set REAR_AUX CAN bit rate: 1000000 | 500000 | 250000 | 100000");
        Serial.println("  base <id>                           set decode base. FL=base FR=+6 RL=+12 RR=+18 (default 0x424)");
        Serial.println("  prog <target> <new_base> <rate> [e] program receiver (A.1), sent at 1 Hz for 11 s");
        Serial.println("                                      target = ID the receiver listens on (its current base, e.g. 0x424)");
        Serial.println("                                      rate: 1=1M 2=500k 3=250k 4=100k   e (emissivity): 0=std 1=default 2=custom");
        Serial.println("  raw on|off                          print every received frame");
        Serial.println("  print on|off                        periodic decoded table");
        Serial.println("  ids                                 list every CAN ID seen since the last clear");
        Serial.println("  clear                               clear seen-ID list");
        Serial.println("  status                              show current settings");
        Serial.println();
    }

    void print_status()
    {
        const auto &ttpms = TTPMSInterfaceInstance::instance();
        Serial.printf("bit rate %lu | decode base 0x%03X | FL 0x%03X FR 0x%03X RL 0x%03X RR 0x%03X | raw %s | config seq %s\n",
                      static_cast<unsigned long>(can_baudrate),
                      ttpms.getBaseCANID(),
                      ttpms.getSensorBaseCANID(TTPMSCorner_e::FL),
                      ttpms.getSensorBaseCANID(TTPMSCorner_e::FR),
                      ttpms.getSensorBaseCANID(TTPMSCorner_e::RL),
                      ttpms.getSensorBaseCANID(TTPMSCorner_e::RR),
                      print_raw ? "on" : "off",
                      config_active ? "ACTIVE" : "idle");
    }

    void print_seen_ids()
    {
        const auto &ttpms = TTPMSInterfaceInstance::instance();
        unsigned long now = millis();
        Serial.printf("\n%u unique IDs seen%s\n", static_cast<unsigned>(num_seen_ids), seen_overflow_count ? " (list full, some dropped)" : "");
        for (size_t i = 0; i < num_seen_ids; i++)
        {
            const SeenID_s &e = seen_ids[i];
            Serial.printf("  0x%03lX%s  n=%-7lu age=%-6lu ms  ", static_cast<unsigned long>(e.id), e.extended ? "x" : " ",
                          static_cast<unsigned long>(e.count), now - e.last_millis);
            for (uint8_t b = 0; b < e.len && b < 8; b++)
            {
                Serial.printf("%02X ", e.buf[b]);
            }
            if (!e.extended && ttpms.isTTPMSMessage(e.id))
            {
                uint32_t offset = e.id - ttpms.getBaseCANID();
                Serial.printf(" <- %s msg %lu", CORNER_NAMES[offset / ttpms_default_params::MSGS_PER_SENSOR],
                              static_cast<unsigned long>((offset % ttpms_default_params::MSGS_PER_SENSOR) + 1));
            }
            Serial.println();
        }
        Serial.println();
    }

    void print_ttpms_table()
    {
        const auto &ttpms = TTPMSInterfaceInstance::instance();
        const TTPMSAllSensorData_s data = ttpms.getTTPMSData();
        unsigned long now = millis();

        Serial.printf("\n--- TTPMS @ %lu ms | %lu bps | base 0x%03X ---\n", now, static_cast<unsigned long>(can_baudrate), ttpms.getBaseCANID());
        Serial.println("     IDs          age_ms   SN     node  batt_mV  HS_P_psi   gauge_psi   RSSI  T_sens  flags");
        for (size_t i = 0; i < ttpms_default_params::NUM_SENSORS; i++)
        {
            const TTPMSSingleSensorData_s s = sensor_at(data, i);
            const uint16_t first_id = ttpms.getSensorBaseCANID(static_cast<TTPMSCorner_e>(i));
            Serial.printf("%s  0x%03X-0x%03X  ", CORNER_NAMES[i], static_cast<unsigned>(first_id),
                          static_cast<unsigned>(first_id + ttpms_default_params::MSGS_PER_SENSOR - 1));

            if (s.last_pressure_recv_millis == 0 && s.last_temp_recv_millis == 0 && s.last_status_recv_millis == 0)
            {
                Serial.println("no data");
                continue;
            }

            if (s.last_pressure_recv_millis != 0)
            {
                Serial.printf("%-7lu  ", now - s.last_pressure_recv_millis);
            }
            else
            {
                Serial.print("-        ");
            }
            Serial.printf("%-5u  %-4u  %-7u  %-9.3f  %-10.2f  %-4d  %-6.1f  %s%s%s\n",
                          s.serial_number, s.node_id, s.battery_voltage_mv, s.pressure_psi, s.gauge_pressure_psi,
                          s.rssi_dbm, s.sensor_temp,
                          s.is_battery_low ? "LOW_BATT " : "",
                          s.is_rssi_weak ? "WEAK_RSSI " : "",
                          ttpms.isSensorActive(s, now, SENSOR_TIMEOUT_MS) ? "" : "STALE");

            if (s.last_temp_recv_millis != 0)
            {
                Serial.print("    IR degC CH1-16:");
                for (size_t ch = 0; ch < ttpms_default_params::TEMP_CHANNELS; ch++)
                {
                    Serial.printf(" %.1f", s.temp_data[ch]);
                }
                Serial.printf("  (age %lu ms)\n", now - s.last_temp_recv_millis);
            }
        }
    }

    void cmd_prog(char *args)
    {
        uint32_t target = 0;
        uint32_t new_base = 0;
        uint32_t rate_code = 0;
        uint32_t emissivity = 1; // default cal

        char *t_target = strtok(args, " ");
        char *t_base = strtok(nullptr, " ");
        char *t_rate = strtok(nullptr, " ");
        char *t_emis = strtok(nullptr, " ");

        if (!parse_uint(t_target, target) || !parse_uint(t_base, new_base) || !parse_uint(t_rate, rate_code) ||
            (t_emis != nullptr && !parse_uint(t_emis, emissivity)))
        {
            Serial.println("usage: prog <target_id> <new_base_id> <rate_code 1-4> [emissivity 0-2]");
            return;
        }

        if (config_active)
        {
            Serial.println("prog: a config sequence is already running");
            return;
        }

        CAN_message_t msg;
        if (!build_base_config_msg(target, new_base, rate_code, emissivity, msg))
        {
            Serial.println("prog: argument out of range");
            return;
        }

        if (new_base > ttpms_default_params::MAX_DECODE_BASE_CAN_ID)
        {
            Serial.println("prog: warning - new base + 23 exceeds 0x7FF, cannot decode all 4 sensors there");
        }

        config_msg = msg;
        config_active = true;
        config_start_millis = millis();
        last_config_send_millis = config_start_millis - CONFIG_SEND_PERIOD_MS; // send on the first poll

        pending_base_id = static_cast<uint16_t>(new_base);
        pending_baudrate = baudrate_from_code(static_cast<uint8_t>(rate_code));

        Serial.printf("prog: sending on 0x%03lX at %lu bps for ~11 s:", static_cast<unsigned long>(target), static_cast<unsigned long>(can_baudrate));
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.printf(" %02X", msg.buf[i]);
        }
        Serial.println();
    }

    void handle_command(char *line)
    {
        char *cmd = strtok(line, " ");
        if (cmd == nullptr)
        {
            return;
        }
        char *rest = strtok(nullptr, "");
        uint32_t value = 0;

        if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0)
        {
            print_help();
        }
        else if (strcmp(cmd, "rate") == 0)
        {
            if (!parse_uint(rest, value) || !is_valid_baudrate(value))
            {
                Serial.println("usage: rate 1000000 | 500000 | 250000 | 100000");
                return;
            }
            set_can_baudrate(value);
        }
        else if (strcmp(cmd, "base") == 0)
        {
            if (!parse_uint(rest, value) || value > 0xFFFF || !TTPMSInterfaceInstance::instance().setBaseCANID(static_cast<uint16_t>(value)))
            {
                Serial.printf("usage: base <id>, 0x001 - 0x%03X\n", ttpms_default_params::MAX_DECODE_BASE_CAN_ID);
                return;
            }
            print_status();
        }
        else if (strcmp(cmd, "prog") == 0)
        {
            if (rest == nullptr)
            {
                Serial.println("usage: prog <target_id> <new_base_id> <rate_code 1-4> [emissivity 0-2]");
                return;
            }
            cmd_prog(rest);
        }
        else if (strcmp(cmd, "raw") == 0)
        {
            print_raw = (rest != nullptr && strcmp(rest, "on") == 0);
            Serial.printf("raw %s\n", print_raw ? "on" : "off");
        }
        else if (strcmp(cmd, "print") == 0)
        {
            print_table = (rest != nullptr && strcmp(rest, "on") == 0);
            Serial.printf("print %s\n", print_table ? "on" : "off");
        }
        else if (strcmp(cmd, "ids") == 0)
        {
            print_seen_ids();
        }
        else if (strcmp(cmd, "clear") == 0)
        {
            num_seen_ids = 0;
            seen_overflow_count = 0;
            Serial.println("seen-ID list cleared");
        }
        else if (strcmp(cmd, "status") == 0)
        {
            print_status();
        }
        else
        {
            Serial.printf("unknown command '%s', type help\n", cmd);
        }
    }

    void poll_serial()
    {
        while (Serial.available() > 0)
        {
            char c = static_cast<char>(Serial.read());
            if (c == '\r' || c == '\n')
            {
                if (line_len > 0)
                {
                    line_buf[line_len] = '\0';
                    Serial.printf("> %s\n", line_buf);
                    handle_command(line_buf);
                    line_len = 0;
                }
            }
            else if (line_len < LINE_BUF_LEN - 1)
            {
                line_buf[line_len++] = c;
            }
        }
    }

    void poll_can()
    {
        auto &can = VCRCANInterfaceInstance::instance();
        while (can.rear_aux_can_rx_buffer.available())
        {
            std::array<uint8_t, CAN_MSG_SIZE> buf;
            can.rear_aux_can_rx_buffer.pop_front(buf.data(), CAN_MSG_SIZE);
            CAN_message_t msg;
            memmove(&msg, buf.data(), sizeof(msg));

            record_seen_id(msg);
            if (print_raw)
            {
                print_raw_msg(msg);
            }
            TTPMSInterfaceInstance::instance().receiveTTPMSData(msg, millis());
        }
    }

    void poll_config_sequence()
    {
        if (!config_active)
        {
            return;
        }

        const unsigned long now = millis();
        if ((now - config_start_millis) > CONFIG_SEND_DURATION_MS)
        {
            // Datasheet: do NOT keep sending to the receiver. Power cycle it to apply the change.
            config_active = false;
            Serial.println("\nprog: DONE. Unplug receiver power for 5 s, then plug it back in.");
            if (TTPMSInterfaceInstance::instance().setBaseCANID(pending_base_id))
            {
                Serial.printf("prog: decode base -> 0x%03X\n", pending_base_id);
            }
            if (pending_baudrate != 0 && pending_baudrate != can_baudrate)
            {
                set_can_baudrate(pending_baudrate);
            }
            print_status();
            return;
        }

        if ((now - last_config_send_millis) >= CONFIG_SEND_PERIOD_MS)
        {
            VCRCANInterfaceInstance::instance().REAR_AUX_CAN.write(config_msg);
            last_config_send_millis = now;
        }
    }
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000) {}

    TTPMSInterfaceInstance::create();
    VCRCANInterfaceInstance::create(
        etl::delegate<void(CANInterfaces_s&, const CAN_message_t&, unsigned long, CANInterfaceType_e)>::create<VCRCANInterfaceImpl::vcr_recv_switch>());
    handle_CAN_setup(VCRCANInterfaceInstance::instance().REAR_AUX_CAN, can_baudrate, &VCRCANInterfaceImpl::on_auxillary_can_receive);

    Serial.println("\n=== VCR TTPMS test (REAR_AUX / CAN2) ===");
    print_status();
    print_help();
}

void loop()
{
    poll_serial();
    poll_can();
    poll_config_sequence();

    unsigned long now = millis();
    if (print_table && (now - last_print_millis) >= PRINT_PERIOD_MS)
    {
        last_print_millis = now;
        print_ttpms_table();
    }
}
