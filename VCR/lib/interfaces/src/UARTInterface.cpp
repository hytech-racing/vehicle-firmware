#include "UARTInterface.hpp"

#include <cmath>
#include <pb_decode.h>

namespace
{
    bool valid_vector(const hytech_msgs_veh_vec_float& values, bool nonnegative)
    {
        const float corners[] = {values.FL, values.FR, values.RL, values.RR};
        for (float value : corners)
        {
            if (!std::isfinite(value) || (nonnegative && value < 0.0f))
                return false;
        }
        return true;
    }
} // namespace

void UARTCommandDecoder::reset()
{
    _framer.reset();
}

bool UARTCommandDecoder::feed(uint8_t byte, uint32_t now_millis, Command& output)
{
    const bool complete = _framer.feed(byte);
    _stats.rx_bytes = _framer.stats().rx_bytes;
    _stats.cobs_errors = _framer.stats().cobs_errors;
    _stats.crc_errors = _framer.stats().crc_errors;
    _stats.oversize_errors = _framer.stats().oversize_errors;
    return complete && _decode(now_millis, output);
}

bool UARTCommandDecoder::is_valid(const hytech_msgs_MCUCommandData& message)
{
    return message.has_desired_rpms && message.has_torque_limit_nm && valid_vector(message.desired_rpms, false) && valid_vector(message.torque_limit_nm, true);
}

bool UARTCommandDecoder::_decode(uint32_t now_millis, Command& output)
{
    Command candidate;
    pb_istream_t stream = pb_istream_from_buffer(_framer.payload(), _framer.payload_size());
    if (!pb_decode(&stream, hytech_msgs_MCUCommandData_fields, &candidate.message))
    {
        ++_stats.protobuf_errors;
        return false;
    }
    const auto& message = candidate.message;
    if (!is_valid(message))
    {
        ++_stats.invalid_commands;
        return false;
    }
    candidate.received_millis = now_millis;
    output = candidate;
    ++_stats.valid_commands;
    return true;
}

void UARTFrameCodec::reset()
{
    _size = 0;
    _discarding = false;
}

bool UARTFrameCodec::feed(uint8_t byte)
{
    ++_stats.rx_bytes;
    if (byte)
    {
        if (!_discarding)
        {
            if (_size == _encoded.size())
            {
                ++_stats.oversize_errors;
                _size = 0;
                _discarding = true;
            }
            else
                _encoded[_size++] = byte;
        }
        return false;
    }
    const bool valid = !_discarding && _size && decode();
    reset();
    return valid;
}

uint16_t UARTFrameCodec::crc16(const uint8_t* bytes, std::size_t size)
{
    uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= static_cast<uint16_t>(bytes[i]) << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

std::size_t UARTFrameCodec::encode(const uint8_t* payload, std::size_t size, uint8_t* output, std::size_t capacity)
{
    if (size > MAX_PAYLOAD_BYTES || capacity < size + 2 + (size + 2) / 254 + 2)
        return 0;
    const auto crc = crc16(payload, size);
    std::size_t code_index = 0, written = 1;
    uint8_t code = 1;
    for (std::size_t i = 0; i < size + 2; ++i)
    {
        const uint8_t byte = i < size ? payload[i] : (i == size ? static_cast<uint8_t>(crc) : static_cast<uint8_t>(crc >> 8));
        if (!byte)
        {
            output[code_index] = code;
            code_index = written++;
            code = 1;
        }
        else
        {
            output[written++] = byte;
            if (++code == 255)
            {
                output[code_index] = code;
                code_index = written++;
                code = 1;
            }
        }
    }
    output[code_index] = code;
    output[written++] = 0;
    return written;
}

bool UARTFrameCodec::decode()
{
    std::size_t input = 0, count = 0;
    while (input < _size)
    {
        const uint8_t code = _encoded[input++];
        if (!code || static_cast<std::size_t>(code - 1) > _size - input)
        {
            ++_stats.cobs_errors;
            return false;
        }
        for (unsigned i = 1; i < code; ++i)
        {
            if (count == _decoded.size())
            {
                ++_stats.oversize_errors;
                return false;
            }
            _decoded[count++] = _encoded[input++];
        }
        if (code != 255 && input < _size)
        {
            if (count == _decoded.size())
            {
                ++_stats.oversize_errors;
                return false;
            }
            _decoded[count++] = 0;
        }
    }
    if (count < 2)
    {
        ++_stats.cobs_errors;
        return false;
    }
    _payload_size = count - 2;
    const auto expected = static_cast<uint16_t>(_decoded[count - 2]) | (static_cast<uint16_t>(_decoded[count - 1]) << 8);
    if (crc16(_decoded.data(), _payload_size) != expected)
    {
        ++_stats.crc_errors;
        return false;
    }
    return true;
}

const UARTFrameCodec::Stats& UARTFrameCodec::stats() const
{
    return _stats;
}

const uint8_t* UARTFrameCodec::payload() const
{
    return _decoded.data();
}

std::size_t UARTFrameCodec::payload_size() const
{
    return _payload_size;
}

const UARTCommandDecoder::Stats& UARTCommandDecoder::stats() const
{
    return _stats;
}

#ifdef ARDUINO
UARTCommandSerial::UARTCommandSerial(HardwareSerial& serial)
    : _serial(serial)
{
}

void UARTCommandSerial::begin(uint32_t baud)
{
    _decoder.reset();
    _serial.begin(baud);
}

bool UARTCommandSerial::poll(uint32_t now_millis, UARTCommandDecoder::Command& output, std::size_t byte_budget)
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

const UARTCommandDecoder::Stats& UARTCommandSerial::stats() const
{
    return _decoder.stats();
}
#endif
