#include "UARTInterface.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <pb_encode.h>
#include <stdexcept>
#include <vector>

using Codec = UARTFrameCodec;

void check(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

std::vector<uint8_t> encode(const hytech_msgs_MCUCommandData& command)
{
    uint8_t payload[hytech_msgs_MCUCommandData_size], frame[Codec::MAX_ENCODED_BYTES + 1];
    auto stream = pb_ostream_from_buffer(payload, sizeof(payload));
    check(pb_encode(&stream, hytech_msgs_MCUCommandData_fields, &command), "protobuf encoding failed");
    const auto size = Codec::encode(payload, stream.bytes_written, frame, sizeof(frame));
    check(size != 0, "frame encoding failed");
    return {frame, frame + size};
}

bool decode(UARTCommandDecoder& decoder, const std::vector<uint8_t>& frame, UARTCommandDecoder::Command& output)
{
    bool accepted = false;
    for (auto byte : frame)
        accepted |= decoder.feed(byte, 42, output);
    return accepted;
}

int main()
try
{
    const uint8_t check_bytes[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    check(Codec::crc16(check_bytes, sizeof(check_bytes)) == 0x29b1, "CRC check failed");

    hytech_msgs_MCUCommandData command = hytech_msgs_MCUCommandData_init_zero;
    command.has_desired_rpms = command.has_torque_limit_nm = true;
    command.desired_rpms = {100, 200, -300, 400};
    command.torque_limit_nm = {1, 2, 3, 4};
    command.prev_MCU_recv_millis = 123;
    auto frame = encode(command);
    check(frame.back() == 0 && std::find(frame.begin(), frame.end() - 1, 0) == frame.end() - 1,
          "invalid COBS framing");
    UARTCommandDecoder decoder;
    UARTCommandDecoder::Command output;
    check(decode(decoder, frame, output), "command decoding failed");
    check(output.message.desired_rpms.RL == -300 && output.message.torque_limit_nm.RR == 4 &&
          output.message.prev_MCU_recv_millis == 123 && output.received_millis == 42, "command values changed");

    frame[frame.size() - 2] ^= 1;
    check(!decode(decoder, frame, output) && decoder.stats().crc_errors == 1, "corrupted checksum accepted");
    check(!decode(decoder, {4, 1, 0}, output) && decoder.stats().cobs_errors == 1, "truncated COBS frame accepted");
    command.torque_limit_nm.FR = -1;
    check(!decode(decoder, encode(command), output), "negative torque allowance accepted");
    command.torque_limit_nm.FR = 2;
    command.desired_rpms.FL = std::numeric_limits<float>::quiet_NaN();
    check(!UARTCommandDecoder::is_valid(command), "nonfinite RPM accepted");
    command.desired_rpms.FL = 100;
    command.has_desired_rpms = false;
    check(!UARTCommandDecoder::is_valid(command), "missing RPM vector accepted");
    command.has_desired_rpms = true;
    check(decode(decoder, encode(command), output), "decoder did not recover after invalid frames");
    std::cout << "UART unit tests passed\n";
    return 0;
}
catch (const std::exception& error)
{
    std::cerr << error.what() << '\n';
    return 1;
}
