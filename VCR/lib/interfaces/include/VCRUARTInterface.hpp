#ifndef VCR_UART_INTERFACE_HPP
#define VCR_UART_INTERFACE_HPP

#include "hytech_msgs.pb.h"

#include <Arduino.h>
#include <array>
#include <cstddef>
#include <cstdint>

// Bounded UART command reception, COBS framing, and CRC-16/CCITT-FALSE validation.
// The caller owns scheduling, freshness, and source selection.
class VCRUARTInterface
{
  public:
    // MCUCommandData is at most 55 bytes; allow room for protobuf unknown fields.
    static constexpr std::size_t MAX_PAYLOAD_BYTES = 128;
    static constexpr std::size_t MAX_ENCODED_BYTES = MAX_PAYLOAD_BYTES + 2 + (MAX_PAYLOAD_BYTES + 2) / 254 + 1;
    struct Command
    {
        hytech_msgs_MCUCommandData message = hytech_msgs_MCUCommandData_init_zero;
        uint32_t received_millis = 0;
    };
    struct Stats
    {
        uint32_t rx_bytes = 0;
        uint32_t valid_commands = 0;
        uint32_t cobs_errors = 0;
        uint32_t crc_errors = 0;
        uint32_t oversize_errors = 0;
        uint32_t protobuf_errors = 0;
        uint32_t invalid_commands = 0;
    };

    explicit VCRUARTInterface(HardwareSerial& serial);
    void begin(uint32_t baud = 921600);
    // Read at most byte_budget bytes, stopping at the first valid command.
    bool poll(uint32_t now_millis, Command& output, std::size_t byte_budget = 256);
    // Feed bytes including the delimiter. Rejections leave output unchanged.
    // now_millis is MCU-local processing time, not a timestamp from the Pi.
    bool feed(uint8_t byte, uint32_t now_millis, Command& output);
    void reset(); // Discard partial input; retain cumulative counters.
    const Stats& stats() const; // Same-thread access only.
    static bool is_valid(const hytech_msgs_MCUCommandData& message);
    static uint16_t crc16(const uint8_t* bytes, std::size_t size);
    // Returns length including delimiter, or zero for oversized input/insufficient storage.
    static std::size_t encode(const uint8_t* payload, std::size_t size, uint8_t* output, std::size_t capacity);

  private:
    bool _decode_frame();
    bool _decode_command(uint32_t now_millis, Command& output);
    HardwareSerial& _serial;
    std::array<uint8_t, MAX_ENCODED_BYTES> _encoded = {};
    std::array<uint8_t, MAX_PAYLOAD_BYTES + 2> _decoded = {};
    std::size_t _size = 0, _payload_size = 0;
    bool _discarding = false;
    Stats _stats;
};

#endif
