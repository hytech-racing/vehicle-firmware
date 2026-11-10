#ifndef UART_COMMAND_SERIAL_HPP
#define UART_COMMAND_SERIAL_HPP

#include "UARTCommandDecoder.hpp"

#include <Arduino.h>

// Hardware access stays outside the decoder, so it can also be tested natively.
class UARTCommandSerial
{
  public:
    explicit UARTCommandSerial(HardwareSerial& serial)
        : _serial(serial)
    {
    }
    void begin(uint32_t baud = 921600)
    {
        _decoder.reset();
        _serial.begin(baud);
    }

    // Consume at most byte_budget bytes and stop at the first valid command.
    // Call again to drain subsequent commands; no command is silently overwritten.
    bool poll(uint32_t now_millis, UARTCommandDecoder::Command& output, std::size_t byte_budget = 256)
    {
        for (std::size_t i = 0; i < byte_budget && _serial.available() > 0; ++i)
        {
            const int byte = _serial.read();
            if (byte < 0)
                break;
            if (_decoder.feed(static_cast<uint8_t>(byte), now_millis, output))
                return true;
        }
        return false;
    }

    const UARTCommandDecoder::Stats& stats() const { return _decoder.stats(); }

  private:
    HardwareSerial& _serial;
    UARTCommandDecoder _decoder;
};

#endif
