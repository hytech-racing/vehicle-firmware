#include "UARTCommandDecoder.hpp"
#include "UARTCommandSerial.hpp"

#include <fstream>
#include <iostream>
#include <limits>
#include <pb_encode.h>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

void require(bool condition, const char* message)
{
    if (!condition)
        throw std::runtime_error(message);
}

std::vector<uint8_t> fixture(const char* name)
{
    std::ifstream file(std::string(FIXTURE_DIR) + "/" + name + ".hex");
    require(static_cast<bool>(file), "missing Drivebrain fixture");
    unsigned byte;
    std::vector<uint8_t> bytes;
    while (file >> std::hex >> byte)
        bytes.push_back(static_cast<uint8_t>(byte));
    return bytes;
}

// Test encoder is independent of the decoder. Golden fixtures additionally come
// from the real Drivebrain driver and are checked by its Python wire reference.
std::vector<uint8_t> frame(std::vector<uint8_t> payload)
{
    uint16_t crc = 0xffff;
    for (uint8_t byte : payload)
    {
        crc ^= static_cast<uint16_t>(byte) << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    payload.push_back(static_cast<uint8_t>(crc));
    payload.push_back(static_cast<uint8_t>(crc >> 8));
    std::vector<uint8_t> encoded(1, 0);
    std::size_t code_index = 0;
    uint8_t code = 1;
    for (auto byte : payload)
    {
        if (byte == 0)
        {
            encoded[code_index] = code;
            code_index = encoded.size();
            encoded.push_back(0);
            code = 1;
        }
        else
        {
            encoded.push_back(byte);
            if (++code == 255)
            {
                encoded[code_index] = code;
                code_index = encoded.size();
                encoded.push_back(0);
                code = 1;
            }
        }
    }
    encoded[code_index] = code;
    encoded.push_back(0);
    return encoded;
}

std::vector<uint8_t> encode(const hytech_msgs_MCUCommandData& command)
{
    uint8_t bytes[hytech_msgs_MCUCommandData_size];
    auto stream = pb_ostream_from_buffer(bytes, sizeof(bytes));
    require(pb_encode(&stream, hytech_msgs_MCUCommandData_fields, &command), "test protobuf encoding failed");
    return frame({bytes, bytes + stream.bytes_written});
}

unsigned feed(UARTCommandDecoder& decoder, const std::vector<uint8_t>& bytes, UARTCommandDecoder::Command& out, uint32_t now = 42)
{
    unsigned count = 0;
    for (auto byte : bytes)
        count += decoder.feed(byte, now, out);
    return count;
}

int main()
try
{
    const auto good = fixture("command");
    const auto zero = fixture("zero");
    UARTCommandDecoder decoder;
    UARTCommandDecoder::Command out;
    require(feed(decoder, good, out, 123) == 1, "Drivebrain command not decoded");
    require(
        out.message.desired_rpms.FL == 100 && out.message.desired_rpms.FR == 200 && out.message.desired_rpms.RL == -300 && out.message.desired_rpms.RR == 400,
        "RPM values/order mismatch");
    require(
        out.message.torque_limit_nm.FL == 1
            && out.message.torque_limit_nm.FR == 2
            && out.message.torque_limit_nm.RL == 3
            && out.message.torque_limit_nm.RR == 4,
        "torque values/order mismatch");
    require(out.message.prev_MCU_recv_millis == 123456 && out.received_millis == 123, "timestamps confused");
    auto valid = out.message;
    auto bad_crc = good;
    bad_crc[2] ^= 1;
    for (const auto& bad : {bad_crc, std::vector<uint8_t>{4, 1, 0}, frame({0x80})})
    {
        require(feed(decoder, bad, out) == 0 && out.received_millis == 123, "bad frame published a command");
    }
    require(decoder.stats().crc_errors == 1 && decoder.stats().cobs_errors == 1 && decoder.stats().protobuf_errors == 1, "wrong frame rejection counters");
    for (int test = 0; test < 5; ++test)
    {
        auto invalid = valid;
        if (test == 0)
            invalid.has_desired_rpms = false;
        if (test == 1)
            invalid.has_torque_limit_nm = false;
        if (test == 2)
            invalid.desired_rpms.FL = std::numeric_limits<float>::quiet_NaN();
        if (test == 3)
            invalid.torque_limit_nm.RR = std::numeric_limits<float>::infinity();
        if (test == 4)
            invalid.torque_limit_nm.FR = -1;
        require(feed(decoder, encode(invalid), out) == 0 && out.received_millis == 123, "invalid command published or refreshed receipt time");
    }
    require(decoder.stats().invalid_commands == 5, "missing command validation counter");
    std::vector<uint8_t> oversized(UARTCommandDecoder::MAX_ENCODED_BYTES + 40, 1);
    oversized.push_back(0);
    require(feed(decoder, oversized, out) == 0 && decoder.stats().oversize_errors == 1, "oversize not rejected once");
    require(
        feed(decoder, zero, out) == 1
            && out.message.has_desired_rpms
            && out.message.has_torque_limit_nm
            && out.message.desired_rpms.FL == 0
            && out.message.torque_limit_nm.RR == 0,
        "zero command/recovery failed");
    // A truncated frame is discarded at its delimiter; the following frame is usable.
    auto truncated = good;
    truncated.resize(5);
    truncated.push_back(0);
    require(feed(decoder, truncated, out) == 0, "truncated frame accepted");
    require(feed(decoder, good, out) == 1, "did not recover after truncation");
    decoder.feed(good.front(), 0, out);
    decoder.reset();
    require(feed(decoder, good, out) == 1, "reset did not clear partial state");

    // An exactly full payload with unknown fields must remain interoperable.
    uint8_t payload[hytech_msgs_MCUCommandData_size];
    auto stream = pb_ostream_from_buffer(payload, sizeof(payload));
    require(pb_encode(&stream, hytech_msgs_MCUCommandData_fields, &valid), "padding fixture encode failed");
    std::vector<uint8_t> padded(payload, payload + stream.bytes_written);
    while (padded.size() + 2 <= UARTCommandDecoder::MAX_PAYLOAD_BYTES)
    {
        padded.push_back(0x78); // Unknown varint field 15.
        padded.push_back(0);
    }
    require(padded.size() == UARTCommandDecoder::MAX_PAYLOAD_BYTES, "padding fixture size wrong");
    require(feed(decoder, frame(padded), out) == 1, "exact-limit payload rejected");
    padded.push_back(0x78);
    require(feed(decoder, frame(padded), out) == 0 && decoder.stats().oversize_errors == 2, "payload limit not enforced");
    // Deterministic noise exercises parser bounds and recovery under sanitizers.
    std::mt19937 random(0x55415254);
    for (unsigned i = 0; i < 20000; ++i)
        decoder.feed(static_cast<uint8_t>(random()), 999, out);
    decoder.feed(0, 999, out);
    require(feed(decoder, good, out) == 1, "did not recover after arbitrary noise");

    HardwareSerial serial;
    UARTCommandSerial adapter(serial);
    adapter.begin();
    require(serial.baud == 921600, "incorrect default baud");
    serial.input = good;
    serial.input.insert(serial.input.end(), zero.begin(), zero.end());
    const auto before = serial.position;
    require(!adapter.poll(10, out, 1) && serial.position == before + 1, "poll work budget ignored");
    require(adapter.poll(11, out) && out.message.desired_rpms.FL == 100, "adapter missed first command");
    require(adapter.poll(12, out) && out.message.desired_rpms.FL == 0, "adapter overwrote second command");
    require(!adapter.poll(13, out) && adapter.stats().valid_commands == 2, "adapter duplicated a command");
    std::cout << "Drivebrain/nanopb compatibility, command validation, bounded framing and serial adapter checks passed\n";
    return 0;
}
catch (const std::exception& error)
{
    std::cerr << error.what() << '\n';
    return 1;
}
