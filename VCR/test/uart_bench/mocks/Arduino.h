#pragma once
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

class HardwareSerialIMXRT
{
  public:
    std::vector<uint8_t> input, output;
    std::size_t position = 0;
    int capacity = 7;
    uint32_t baud = 0;
    void begin(uint32_t value) { baud = value; }
    void end()
    {
        input.clear();
        position = 0;
    }
    void clear()
    {
        input.clear();
        position = 0;
    }
    void addMemoryForRead(void*, std::size_t) {}
    int available() const { return static_cast<int>(input.size() - position); }
    int read() { return available() ? input[position++] : -1; }
    int availableForWrite() const { return capacity; }
    std::size_t write(const uint8_t* bytes, std::size_t count)
    {
        count = std::min(count, static_cast<std::size_t>(capacity));
        output.insert(output.end(), bytes, bytes + count);
        return count;
    }
};
using HardwareSerial = HardwareSerialIMXRT;
inline HardwareSerialIMXRT Serial1, Serial2, Serial3, Serial4, Serial5, Serial6, Serial7, Serial8;
inline uint32_t micros()
{
    static uint32_t value = 0;
    return value += 10;
}
inline uint32_t millis()
{
    return micros() / 1000;
}
