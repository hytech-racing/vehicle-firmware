// Bare Teensy receive example. This entrypoint never starts vehicle systems.
#include "UARTCommandSerial.hpp"

#include <cstdio>

namespace
{
    UARTCommandSerial uart(Serial1); // RX pin 0, TX pin 1; USB Serial is separate.
    UARTCommandDecoder::Command latest;
    uint32_t last_status_millis = 0;
} // namespace

void setup()
{
    Serial.begin(115200);
    uart.begin(921600);
}

void loop()
{
    // One bounded poll per loop. No waiting for USB or for a whole UART frame.
    uart.poll(millis(), latest);
    const uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_status_millis) >= 1000)
    {
        last_status_millis = now;
        // Aggregate diagnostics only; command data is available in latest.message.
        char line[128];
        const int length = snprintf(
            line,
            sizeof(line),
            "valid=%lu crc=%lu cobs=%lu proto=%lu invalid=%lu size=%lu\n",
            static_cast<unsigned long>(uart.stats().valid_commands),
            static_cast<unsigned long>(uart.stats().crc_errors),
            static_cast<unsigned long>(uart.stats().cobs_errors),
            static_cast<unsigned long>(uart.stats().protobuf_errors),
            static_cast<unsigned long>(uart.stats().invalid_commands),
            static_cast<unsigned long>(uart.stats().oversize_errors));
        if (length > 0 && static_cast<std::size_t>(length) < sizeof(line) && Serial.availableForWrite() >= length)
        {
            Serial.write(reinterpret_cast<const uint8_t*>(line), static_cast<std::size_t>(length));
        }
    }
}
