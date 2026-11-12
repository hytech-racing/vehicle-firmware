#include "UARTBenchEndpoint.hpp"

#include "uart_bench_schema.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <pb_decode.h>
#include <pb_encode.h>

namespace
{
    std::array<std::array<uint8_t, 4096>, 8> rx_memory;
    std::array<bool, 8> memory_added{};
    HardwareSerialIMXRT* port_for(unsigned port)
    {
        switch (port)
        {
        case 1:
            return &Serial1;
        case 2:
            return &Serial2;
        case 3:
            return &Serial3;
        case 4:
            return &Serial4;
        case 5:
            return &Serial5;
        case 6:
            return &Serial6;
        case 7:
            return &Serial7;
        case 8:
            return &Serial8;
        default:
            return nullptr;
        }
    }
} // namespace

bool UARTBenchEndpoint::configure(unsigned port, uint32_t baud, bool raw, uint64_t run_id)
{
    auto* serial = port_for(port);
    if (!serial || baud < 9600 || baud > 3000000 || !run_id)
        return false;
    stop();
    _port = port;
    _baud = baud;
    _run_id = run_id;
    _raw = raw;
    _serial = serial;
    _frames = Frames{};
    _raw_uart.reset();
    _commands = _patterns = _protobuf_errors = _invalid_requests = _invalid_commands = _tx_frames = _tx_busy = _processing_max_us = 0;
    if (!memory_added[port - 1])
    {
        _serial->addMemoryForRead(rx_memory[port - 1].data(), rx_memory[port - 1].size());
        memory_added[port - 1] = true;
    }
    if (_raw)
    {
        _raw_uart.emplace(*_serial);
        _raw_uart->begin(baud);
    }
    else
        _serial->begin(baud);
    _serial->clear();
    return true;
}

void UARTBenchEndpoint::stop()
{
    if (_serial)
        _serial->end();
    _serial = nullptr;
    _head = _tail = _queued = 0;
}

void UARTBenchEndpoint::queue(const pb_msgdesc_t* descriptor, const void* message)
{
    if (_queued == _tx.size())
    {
        ++_tx_busy;
        return;
    }
    auto stream = pb_ostream_from_buffer(_payload.data(), _payload.size());
    if (!pb_encode(&stream, descriptor, message))
    {
        ++_protobuf_errors;
        return;
    }
    auto& tx = _tx[_tail];
    tx.size = Frames::encode(_payload.data(), stream.bytes_written, tx.bytes.data(), tx.bytes.size());
    tx.offset = 0;
    if (!tx.size)
    {
        ++_tx_busy;
        return;
    }
    _tail = (_tail + 1) % _tx.size();
    ++_queued;
}

void UARTBenchEndpoint::transmit()
{
    if (!_serial || !_queued)
        return;
    auto& tx = _tx[_head];
    const auto available = static_cast<std::size_t>(_serial->availableForWrite());
    const auto size = std::min(available, tx.size - tx.offset);
    if (size)
        tx.offset += _serial->write(tx.bytes.data() + tx.offset, size);
    if (tx.offset == tx.size)
    {
        _head = (_head + 1) % _tx.size();
        --_queued;
        ++_tx_frames;
    }
}

void UARTBenchEndpoint::process()
{
    const uint32_t started = micros();
    uart_bench_Request request = uart_bench_Request_init_zero;
    auto stream = pb_istream_from_buffer(_frames.payload(), _frames.payload_size());
    if (!pb_decode(&stream, uart_bench_Request_fields, &request))
    {
        ++_protobuf_errors;
        return;
    }
    if (request.version != 1 || request.run_id != _run_id || !request.sequence)
    {
        ++_invalid_requests;
        return;
    }
    uart_bench_Reply reply = uart_bench_Reply_init_zero;
    reply.version = 1;
    reply.run_id = request.run_id;
    reply.sequence = request.sequence;
    if (request.which_body == uart_bench_Request_command_tag)
    {
        if (!UARTCommandDecoder::is_valid(request.body.command))
        {
            ++_invalid_commands;
            return;
        }
        reply.which_body = uart_bench_Reply_command_tag;
        reply.body.command = request.body.command;
        ++_commands;
    }
    else if (request.which_body == uart_bench_Request_pattern_tag)
    {
        reply.which_body = uart_bench_Reply_pattern_tag;
        reply.body.pattern.size = request.body.pattern.size;
        memcpy(reply.body.pattern.bytes, request.body.pattern.bytes, request.body.pattern.size);
        ++_patterns;
    }
    else
    {
        ++_invalid_requests;
        return;
    }
    reply.processing_us = static_cast<uint32_t>(micros() - started);
    _processing_max_us = std::max(_processing_max_us, reply.processing_us);
    queue(uart_bench_Reply_fields, &reply);
}

void UARTBenchEndpoint::tick()
{
    if (!_serial)
        return;
    transmit();
    if (_raw)
    {
        std::size_t remaining = 2048;
        while (remaining && _serial->available())
        {
            UARTCommandDecoder::Command command;
            const uint32_t before = _raw_uart->stats().rx_bytes;
            const bool valid = _raw_uart->poll(millis(), command, remaining);
            const uint32_t consumed = _raw_uart->stats().rx_bytes - before;
            remaining -= consumed;
            if (valid)
            {
                ++_commands;
                queue(hytech_msgs_MCUCommandData_fields, &command.message);
                transmit();
            }
            if (!valid || !consumed)
                break;
        }
        return;
    }
    for (unsigned i = 0; i < 2048 && _serial->available(); ++i)
    {
        const int byte = _serial->read();
        if (byte < 0)
            break;
        if (_frames.feed(static_cast<uint8_t>(byte)))
            process();
        transmit();
    }
}

void UARTBenchEndpoint::status(char* output, std::size_t capacity, const char* event) const
{
    const auto& f = _frames.stats();
    const UARTCommandDecoder::Stats empty{};
    const auto& r = _raw_uart ? _raw_uart->stats() : empty;
    // Teensy's smallest-code printf does not support %llu. Format the session
    // identifier explicitly so subsequent varargs are not misinterpreted.
    char run_text[21];
    std::size_t digits = 0;
    auto value = _run_id;
    do
    {
        run_text[digits++] = static_cast<char>('0' + value % 10);
        value /= 10;
    } while (value);
    std::reverse(run_text, run_text + digits);
    run_text[digits] = 0;
    snprintf(
        output,
        capacity,
        "{\"event\":\"%s\",\"version\":1,\"schema_sha256\":\"%s\",\"port\":%u,\"baud\":%lu,"
        "\"mode\":\"%s\",\"run_id\":\"%s\",\"uptime_ms\":%lu,\"rx_bytes\":%lu,\"commands\":%lu,\"patterns\":%lu,"
        "\"cobs_errors\":%lu,\"crc_errors\":%lu,\"oversize_errors\":%lu,\"protobuf_errors\":%lu,"
        "\"invalid_requests\":%lu,\"invalid_commands\":%lu,\"tx_frames\":%lu,\"tx_busy\":%lu,\"queued\":%u,\"processing_max_us\":%lu}",
        event,
        UART_BENCH_SCHEMA_SHA256,
        _port,
        static_cast<unsigned long>(_baud),
        _raw ? "raw" : "bench",
        run_text,
        static_cast<unsigned long>(millis()),
        static_cast<unsigned long>(_raw ? r.rx_bytes : f.rx_bytes),
        static_cast<unsigned long>(_commands),
        static_cast<unsigned long>(_patterns),
        static_cast<unsigned long>(_raw ? r.cobs_errors : f.cobs_errors),
        static_cast<unsigned long>(_raw ? r.crc_errors : f.crc_errors),
        static_cast<unsigned long>(_raw ? r.oversize_errors : f.oversize_errors),
        static_cast<unsigned long>(_protobuf_errors + (_raw ? r.protobuf_errors : 0)),
        static_cast<unsigned long>(_invalid_requests),
        static_cast<unsigned long>(_invalid_commands + (_raw ? r.invalid_commands : 0)),
        static_cast<unsigned long>(_tx_frames),
        static_cast<unsigned long>(_tx_busy),
        static_cast<unsigned>(_queued),
        static_cast<unsigned long>(_processing_max_us));
}
