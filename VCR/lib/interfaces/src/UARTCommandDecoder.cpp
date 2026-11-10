#include "UARTCommandDecoder.hpp"

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
