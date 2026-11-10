#ifndef UART_COMMAND_DECODER_HPP
#define UART_COMMAND_DECODER_HPP

#include "UARTFrameCodec.hpp"
#include "hytech_msgs.pb.h"

#include <array>
#include <cstddef>
#include <cstdint>

// Pure software decoder. The caller owns scheduling, freshness, and source selection.
// No global VCR state or actuator interface is accessed here.
class UARTCommandDecoder
{
  public:
    // MCUCommandData is at most 55 bytes with the pinned schema. Leave room for
    // protobuf unknown fields, while keeping memory and parser work bounded.
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

    // Feed bytes in order, including trailing zero delimiters. Returns true only
    // for a complete validated command; output is unchanged for every rejection.
    // now_millis is the MCU's local processing time, not a timestamp from the Pi.
    bool feed(uint8_t byte, uint32_t now_millis, Command& output);
    static bool is_valid(const hytech_msgs_MCUCommandData& message);
    void reset();                                 // Discard partial input; retain cumulative counters.
    const Stats& stats() const { return _stats; } // Same-thread access only.

  private:
    bool _decode(uint32_t now_millis, Command& output);
    UARTFrameCodec<MAX_PAYLOAD_BYTES> _framer;
    Stats _stats;
};

#endif
