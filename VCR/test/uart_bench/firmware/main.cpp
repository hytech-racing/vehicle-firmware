// Bare Teensy only. No vehicle state, CAN, external watchdog, or motor control.
#include "UARTBenchEndpoint.hpp"

#include <cstdio>
#include <cstring>

namespace
{
    UARTBenchEndpoint endpoint;
    char input[192]{};
    std::size_t input_size = 0;
    bool discard_input = false;
    struct UsbLine
    {
        char bytes[1024];
        std::size_t size = 0, offset = 0;
    };
    std::array<UsbLine, 4> output;
    std::size_t head = 0, tail = 0, queued = 0;
    uint32_t last_status = 0;

    void respond(const char* event)
    {
        if (queued == output.size())
            return; // USB backpressure must not block UART.
        auto& line = output[tail];
        endpoint.status(line.bytes, sizeof(line.bytes) - 1, event);
        line.size = strlen(line.bytes);
        line.bytes[line.size++] = '\n';
        line.offset = 0;
        tail = (tail + 1) % output.size();
        ++queued;
    }

    void command()
    {
        unsigned port;
        unsigned long baud;
        char run_text[21]{};
        uint64_t run = 0;
        char mode[16]{};
        char extra;
        const auto parse_run = [&]()
        {
            if (!run_text[0])
                return false;
            for (const char* p = run_text; *p; ++p)
            {
                if (*p < '0' || *p > '9')
                    return false;
                const auto digit = static_cast<uint64_t>(*p - '0');
                if (run > (UINT64_MAX - digit) / 10)
                    return false;
                run = run * 10 + digit;
            }
            return run != 0;
        };
        if (!strcmp(input, "HELLO"))
            respond("identity");
        else if (!strcmp(input, "STATUS"))
            respond("snapshot");
        else if (!strcmp(input, "STOP"))
        {
            endpoint.stop();
            respond("stopped");
        }
        else if (
            sscanf(input, "CONFIG %u %lu %15s %20s %c", &port, &baud, mode, run_text, &extra) == 4
            && (!strcmp(mode, "raw") || !strcmp(mode, "bench"))
            && parse_run()
            && endpoint.configure(port, baud, !strcmp(mode, "raw"), run))
            respond("configured");
        else
            respond("control_error");
    }
} // namespace

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    endpoint.tick();
    for (unsigned i = 0; i < 256 && Serial.available(); ++i)
    {
        const char byte = static_cast<char>(Serial.read());
        if (byte == '\r')
            continue;
        if (byte == '\n')
        {
            if (!discard_input && input_size)
            {
                input[input_size] = 0;
                command();
            }
            input_size = 0;
            discard_input = false;
        }
        else if (!discard_input)
        {
            if (input_size + 1 == sizeof(input))
                discard_input = true;
            else
                input[input_size++] = byte;
        }
    }
    if (static_cast<uint32_t>(millis() - last_status) >= 1000)
    {
        last_status = millis();
        if (queued < 3)
            respond("status"); // Leave space for control responses.
    }
    if (queued)
    {
        auto& line = output[head];
        const auto available = static_cast<std::size_t>(Serial.availableForWrite());
        const auto size = std::min(available, line.size - line.offset);
        if (size)
            line.offset += Serial.write(reinterpret_cast<const uint8_t*>(line.bytes + line.offset), size);
        if (line.offset == line.size)
        {
            head = (head + 1) % output.size();
            --queued;
        }
    }
}
