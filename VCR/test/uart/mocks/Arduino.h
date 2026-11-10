#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

class HardwareSerial
{
  public:
    uint32_t baud = 0;
    std::vector<uint8_t> input;
    std::size_t position = 0;
    void begin(uint32_t value) { baud = value; }
    int available() { return static_cast<int>(input.size() - position); }
    int read() { return available() ? input[position++] : -1; }
};
