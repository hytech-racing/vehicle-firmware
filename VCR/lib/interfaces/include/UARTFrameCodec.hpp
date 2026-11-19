#ifndef UART_FRAME_CODEC_HPP
#define UART_FRAME_CODEC_HPP

#include <array>
#include <cstddef>
#include <cstdint>

// Bounded COBS framing with a CRC-16/CCITT-FALSE checksum.
// No allocation, serial peripheral, or protobuf dependency is required here.
template <std::size_t MaxPayload>
class UARTFrameCodec
{
  public:
    static constexpr std::size_t MAX_ENCODED_BYTES = MaxPayload + 2 + (MaxPayload + 2) / 254 + 1;
    struct Stats
    {
        uint32_t rx_bytes = 0;
        uint32_t cobs_errors = 0;
        uint32_t crc_errors = 0;
        uint32_t oversize_errors = 0;
    };

    void reset()
    {
        _size = 0;
        _discarding = false;
    }
    const Stats& stats() const { return _stats; }
    const uint8_t* payload() const { return _decoded.data(); }
    std::size_t payload_size() const { return _payload_size; }

    // A decoded payload remains available until the next feed/reset. Empty
    // delimiters are harmless; malformed/oversize input resynchronizes at zero.
    bool feed(uint8_t byte)
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

    static uint16_t crc16(const uint8_t* bytes, std::size_t size)
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

    // Returns encoded length including delimiter, or zero if storage is too small.
    static std::size_t encode(const uint8_t* payload, std::size_t size, uint8_t* output, std::size_t capacity)
    {
        if (size > MaxPayload || capacity < size + 2 + (size + 2) / 254 + 2)
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

  private:
    bool decode()
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
    std::array<uint8_t, MAX_ENCODED_BYTES> _encoded = {};
    std::array<uint8_t, MaxPayload + 2> _decoded = {};
    std::size_t _size = 0, _payload_size = 0;
    bool _discarding = false;
    Stats _stats;
};
#endif
