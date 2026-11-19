#pragma once

#include <cstdint>

// Native tests compile the serial adapter without Arduino or serial I/O.
class HardwareSerial
{
  public:
    virtual ~HardwareSerial() = default;
    virtual void begin(uint32_t baud) = 0;
    virtual int available() = 0;
    virtual int read() = 0;
};
