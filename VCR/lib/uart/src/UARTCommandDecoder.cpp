#include "UARTCommandDecoder.hpp"

#include <cmath>
#include <pb_decode.h>

namespace
{
    uint16_t crc16(const uint8_t* bytes, std::size_t size)
    {
        uint16_t crc = 0xffff;
        for (std::size_t i = 0; i < size; ++i)
        {
            crc ^= static_cast<uint16_t>(bytes[i]) << 8;
            for (unsigned bit = 0; bit < 8; ++bit)
            {
                crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }

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
    _encoded_size = 0;
    _discarding = false;
}

bool UARTCommandDecoder::feed(uint8_t byte, uint32_t now_millis, Command& output)
{
    ++_stats.rx_bytes;
    if (byte != 0)
    {
        if (!_discarding)
        {
            if (_encoded_size == _encoded.size())
            {
                ++_stats.oversize_errors;
                _encoded_size = 0;
                _discarding = true;
            }
            else
                _encoded[_encoded_size++] = byte;
        }
        return false;
    }
    const bool accepted = !_discarding && _encoded_size != 0 && _decode(now_millis, output);
    reset();
    return accepted;
}

bool UARTCommandDecoder::_decode(uint32_t now_millis, Command& output)
{
    std::size_t input = 0;
    std::size_t decoded_size = 0;
    while (input < _encoded_size)
    {
        const uint8_t code = _encoded[input++];
        if (code == 0 || static_cast<std::size_t>(code - 1) > _encoded_size - input)
        {
            ++_stats.cobs_errors;
            return false;
        }
        for (unsigned i = 1; i < code; ++i)
        {
            if (decoded_size == _decoded.size())
            {
                ++_stats.oversize_errors;
                return false;
            }
            _decoded[decoded_size++] = _encoded[input++];
        }
        if (code != 0xff && input < _encoded_size)
        {
            if (decoded_size == _decoded.size())
            {
                ++_stats.oversize_errors;
                return false;
            }
            _decoded[decoded_size++] = 0;
        }
    }
    if (decoded_size < 2)
    {
        ++_stats.cobs_errors;
        return false;
    }
    const std::size_t payload_size = decoded_size - 2;
    const uint16_t received_crc = static_cast<uint16_t>(_decoded[payload_size]) | (static_cast<uint16_t>(_decoded[payload_size + 1]) << 8);
    if (crc16(_decoded.data(), payload_size) != received_crc)
    {
        ++_stats.crc_errors;
        return false;
    }

    Command candidate;
    pb_istream_t stream = pb_istream_from_buffer(_decoded.data(), payload_size);
    if (!pb_decode(&stream, hytech_msgs_MCUCommandData_fields, &candidate.message))
    {
        ++_stats.protobuf_errors;
        return false;
    }
    const auto& message = candidate.message;
    if (!message.has_desired_rpms || !message.has_torque_limit_nm || !valid_vector(message.desired_rpms, false) || !valid_vector(message.torque_limit_nm, true))
    {
        ++_stats.invalid_commands;
        return false;
    }
    candidate.received_millis = now_millis;
    output = candidate;
    ++_stats.valid_commands;
    return true;
}
