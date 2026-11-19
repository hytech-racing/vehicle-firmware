#include "VCRUARTInterface.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <pb_encode.h>

TEST(VCRUARTInterfaceTesting, crc_check_vector)
{
    const uint8_t bytes[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(VCRUARTInterface::crc16(bytes, sizeof(bytes)), 0x29b1);
}

TEST(VCRUARTInterfaceTesting, command_round_trip_and_reject_invalid_frames)
{
    hytech_msgs_MCUCommandData command = hytech_msgs_MCUCommandData_init_zero;
    command.has_desired_rpms = command.has_torque_limit_nm = true;
    command.desired_rpms = {100, 200, -300, 400};
    command.torque_limit_nm = {1, 2, 3, 4};
    command.prev_MCU_recv_millis = 123;
    uint8_t payload[hytech_msgs_MCUCommandData_size], frame[VCRUARTInterface::MAX_ENCODED_BYTES + 1];
    auto stream = pb_ostream_from_buffer(payload, sizeof(payload));
    ASSERT_TRUE(pb_encode(&stream, hytech_msgs_MCUCommandData_fields, &command));
    const auto size = VCRUARTInterface::encode(payload, stream.bytes_written, frame, sizeof(frame));
    ASSERT_GT(size, 0u);
    EXPECT_EQ(frame[size - 1], 0);

    // Decode in memory without starting or accessing the serial port.
    VCRUARTInterface uart(Serial1);
    VCRUARTInterface::Command output;
    const auto decode = [&]() {
        bool accepted = false;
        for (std::size_t i = 0; i < size; ++i)
            accepted |= uart.feed(frame[i], 42, output);
        return accepted;
    };
    ASSERT_TRUE(decode());
    EXPECT_FLOAT_EQ(output.message.desired_rpms.RL, -300);
    EXPECT_FLOAT_EQ(output.message.torque_limit_nm.RR, 4);
    EXPECT_EQ(output.message.prev_MCU_recv_millis, 123);
    EXPECT_EQ(output.received_millis, 42u);
    frame[size - 2] ^= 1;
    EXPECT_FALSE(decode());
    EXPECT_FALSE(uart.feed(4, 42, output));
    EXPECT_FALSE(uart.feed(1, 42, output));
    EXPECT_FALSE(uart.feed(0, 42, output));
    frame[size - 2] ^= 1;
    EXPECT_TRUE(decode());
}

TEST(VCRUARTInterfaceTesting, command_validation)
{
    hytech_msgs_MCUCommandData command = hytech_msgs_MCUCommandData_init_zero;
    command.has_desired_rpms = command.has_torque_limit_nm = true;
    EXPECT_TRUE(VCRUARTInterface::is_valid(command));
    command.torque_limit_nm.FR = -1;
    EXPECT_FALSE(VCRUARTInterface::is_valid(command));
    command.torque_limit_nm.FR = 0;
    command.desired_rpms.FL = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(VCRUARTInterface::is_valid(command));
    command.desired_rpms.FL = 0;
    command.has_desired_rpms = false;
    EXPECT_FALSE(VCRUARTInterface::is_valid(command));
}
