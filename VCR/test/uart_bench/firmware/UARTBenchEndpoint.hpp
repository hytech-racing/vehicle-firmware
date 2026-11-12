#pragma once
#include "UARTCommandDecoder.hpp"
#include "UARTFrameCodec.hpp"
#include "uart_bench.pb.h"

#include <Arduino.h>

class UARTBenchEndpoint
{
  public:
    bool configure(unsigned port, uint32_t baud, bool raw, uint64_t run_id);
    void tick();
    void stop();
    void status(char* output, std::size_t capacity, const char* event = "status") const;

  private:
    using Frames = UARTFrameCodec<1536>;
    struct Tx
    {
        std::array<uint8_t, Frames::MAX_ENCODED_BYTES + 1> bytes;
        std::size_t size = 0, offset = 0;
    };
    void process();
    void transmit();
    void queue(const pb_msgdesc_t* descriptor, const void* message);
    HardwareSerialIMXRT* _serial = nullptr;
    Frames _frames;
    UARTCommandDecoder _raw_decoder;
    std::array<Tx, 8> _tx;
    std::array<uint8_t, 1536> _payload;
    std::size_t _head = 0, _tail = 0, _queued = 0;
    uint64_t _run_id = 0;
    uint32_t _baud = 0;
    unsigned _port = 0;
    bool _raw = false;
    uint32_t _commands = 0, _patterns = 0, _protobuf_errors = 0, _invalid_requests = 0, _invalid_commands = 0, _tx_frames = 0, _tx_busy = 0,
             _processing_max_us = 0;
};
