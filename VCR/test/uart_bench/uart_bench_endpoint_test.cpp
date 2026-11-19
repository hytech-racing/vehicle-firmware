#include "UARTBenchEndpoint.hpp"

#include <fstream>
#include <iostream>
#include <pb_decode.h>
#include <pb_encode.h>
#include <stdexcept>
#include <string>

using Frames = UARTFrameCodec<1536>;
void require(bool value, const char* error)
{
    if (!value)
        throw std::runtime_error(error);
}

std::vector<uint8_t> encode(const pb_msgdesc_t* fields, const void* message)
{
    uint8_t payload[1536], encoded[Frames::MAX_ENCODED_BYTES + 1];
    auto stream = pb_ostream_from_buffer(payload, sizeof(payload));
    require(pb_encode(&stream, fields, message), "protobuf test encoding failed");
    const auto size = Frames::encode(payload, stream.bytes_written, encoded, sizeof(encoded));
    require(size != 0, "frame encoding failed");
    return {encoded, encoded + size};
}

void deliver(UARTBenchEndpoint& endpoint, const std::vector<uint8_t>& frame)
{
    Serial1.input.insert(Serial1.input.end(), frame.begin(), frame.end());
    for (unsigned i = 0; i < 1000; ++i)
        endpoint.tick();
}

uart_bench_Reply reply()
{
    Frames frames;
    uart_bench_Reply out = uart_bench_Reply_init_zero;
    unsigned count = 0;
    for (auto byte : Serial1.output)
    {
        if (!frames.feed(byte))
            continue;
        auto stream = pb_istream_from_buffer(frames.payload(), frames.payload_size());
        require(pb_decode(&stream, uart_bench_Reply_fields, &out), "reply nanopb decode failed");
        ++count;
    }
    Serial1.output.clear();
    require(count == 1, "expected one complete reply despite partial writes");
    return out;
}

int main()
try
{
    UARTBenchEndpoint endpoint;
    constexpr uint64_t run = UINT64_MAX;
    require(endpoint.configure(1, 921600, false, run), "configuration failed");
    uart_bench_Request request = uart_bench_Request_init_zero;
    request.version = 1;
    request.run_id = run;
    request.sequence = 1;
    request.which_body = uart_bench_Request_command_tag;
    request.body.command.has_desired_rpms = true;
    request.body.command.has_torque_limit_nm = true;
    request.body.command.desired_rpms = {100, 200, -300, 400};
    request.body.command.torque_limit_nm = {1, 2, 3, 4};
    request.body.command.prev_MCU_recv_millis = 123456;
    deliver(endpoint, encode(uart_bench_Request_fields, &request));
    auto out = reply();
    require(out.run_id == run && out.sequence == 1 && out.version == 1, "reply metadata mismatch");
    require(
        out.which_body == uart_bench_Reply_command_tag
            && out.body.command.desired_rpms.RL == -300
            && out.body.command.torque_limit_nm.RR == 4
            && out.body.command.prev_MCU_recv_millis == 123456,
        "decoded values changed");
    char status[1024];
    endpoint.status(status, sizeof(status));
    require(std::string(status).find("\"run_id\":\"18446744073709551615\"") != std::string::npos, "64-bit ID formatting failed");

    request.which_body = uart_bench_Request_pattern_tag;
    for (unsigned size : {0U, 254U, 1024U})
    {
        request.body.pattern.size = size;
        for (unsigned i = 0; i < size; ++i)
            request.body.pattern.bytes[i] = i & 255;
        deliver(endpoint, encode(uart_bench_Request_fields, &request));
        out = reply();
        require(out.which_body == uart_bench_Reply_pattern_tag && out.body.pattern.size == size, "pattern size changed");
        for (unsigned i = 0; i < size; ++i)
            require(out.body.pattern.bytes[i] == (i & 255), "binary byte changed");
    }
    request.run_id = 1;
    deliver(endpoint, encode(uart_bench_Request_fields, &request));
    require(Serial1.output.empty(), "wrong session accepted");
    endpoint.status(status, sizeof(status));
    require(std::string(status).find("\"invalid_requests\":1") != std::string::npos, "missing rejection counter");

    require(endpoint.configure(1, 921600, true, run), "raw mode failed");
    std::ifstream fixture(std::string(FIXTURE_DIR) + "/command.hex");
    require(static_cast<bool>(fixture), "missing production fixture");
    std::vector<uint8_t> raw;
    unsigned byte;
    while (fixture >> std::hex >> byte)
        raw.push_back(static_cast<uint8_t>(byte));
    deliver(endpoint, raw);
    require(Serial1.output == raw, "raw production decode/re-encode differs from Drivebrain fixture");
    Serial1.output.clear();

    // Several frames in one read burst must all pass through UARTCommandSerial.
    std::vector<uint8_t> burst;
    for (unsigned i = 0; i < 4; ++i)
        burst.insert(burst.end(), raw.begin(), raw.end());
    deliver(endpoint, burst);
    require(Serial1.output == burst, "raw burst lost or reordered commands");
    Serial1.output.clear();
    endpoint.status(status, sizeof(status));
    require(std::string(status).find("\"commands\":5") != std::string::npos, "raw command count incorrect");

    // Reconfiguration must discard partial input and reset command statistics.
    Serial1.input.insert(Serial1.input.end(), raw.begin(), raw.begin() + raw.size() / 2);
    endpoint.tick();
    require(endpoint.configure(1, 921600, true, run), "raw reconfiguration failed");
    endpoint.status(status, sizeof(status));
    require(std::string(status).find("\"commands\":0") != std::string::npos, "raw command count was not reset");
    deliver(endpoint, raw);
    require(Serial1.output == raw, "raw decoder did not recover after reconfiguration");
    Serial1.output.clear();

    require(endpoint.configure(1, 921600, false, run), "queue test configuration failed");
    Serial1.capacity = 0;
    request.run_id = run;
    request.body.pattern.size = 0;
    for (unsigned i = 0; i < 20; ++i)
        deliver(endpoint, encode(uart_bench_Request_fields, &request));
    endpoint.status(status, sizeof(status));
    require(std::string(status).find("\"tx_busy\":12") != std::string::npos, "bounded reply queue not enforced");
    Serial1.capacity = 7;
    for (unsigned i = 0; i < 1000; ++i)
        endpoint.tick();
    endpoint.status(status, sizeof(status));
    require(std::string(status).find("\"queued\":0") != std::string::npos, "queue did not drain after recovery");
    endpoint.stop();
    std::cout << "Typed bench decode/replies, raw compatibility, pattern boundaries, session validation and partial TX passed\n";
    return 0;
}
catch (const std::exception& error)
{
    std::cerr << error.what() << '\n';
    return 1;
}
