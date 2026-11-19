#pragma once

#include <cstdint>
#include <stdexcept>

// Native tests can construct serial-dependent interfaces, but cannot perform I/O.
class HardwareSerial
{
  public:
    void begin(uint32_t) { throw std::logic_error("unexpected serial access in native test"); }
    int available() { throw std::logic_error("unexpected serial access in native test"); }
    int read() { throw std::logic_error("unexpected serial access in native test"); }
};

inline HardwareSerial Serial1;
