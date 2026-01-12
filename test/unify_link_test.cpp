/**
 * @file unify_link_test.cpp
 * @brief Unit tests for Unify_link core functionality
 */

#include "encoder_link.hpp"
#include "motor_link.hpp"
#include "unify_link.hpp"

#include <algorithm>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

using namespace unify_link;

// ============================================================================
// Circular Buffer Tests
// ============================================================================

class CircularBufferTest : public ::testing::Test
{
protected:
    static constexpr uint32_t BUFFER_SIZE = 256;
    Circular_buffer<uint8_t, BUFFER_SIZE> buffer;
};

TEST_F(CircularBufferTest, InitialState)
{
    EXPECT_EQ(buffer.used(), 0u);
    EXPECT_EQ(buffer.remain(), BUFFER_SIZE - 1);
}

TEST_F(CircularBufferTest, PushAndReadData)
{
    uint8_t input[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint8_t output[5] = {0};

    EXPECT_EQ(buffer.push_data(input, 5), 5u);
    EXPECT_EQ(buffer.used(), 5u);

    EXPECT_EQ(buffer.read_data(output, 5), 5u);
    EXPECT_EQ(memcmp(input, output, 5), 0);

    // read_data doesn't consume data
    EXPECT_EQ(buffer.used(), 5u);
}

TEST_F(CircularBufferTest, PushAndPopData)
{
    uint8_t input[] = {0xAA, 0xBB, 0xCC};

    buffer.push_data(input, 3);
    EXPECT_EQ(buffer.used(), 3u);

    buffer.pop_data(2);
    EXPECT_EQ(buffer.used(), 1u);

    buffer.pop_data(1);
    EXPECT_EQ(buffer.used(), 0u);
}

TEST_F(CircularBufferTest, ReadWithOffset)
{
    uint8_t input[] = {0x10, 0x20, 0x30, 0x40, 0x50};
    uint8_t output[2] = {0};

    buffer.push_data(input, 5);

    // Read 2 bytes starting from offset 2
    EXPECT_EQ(buffer.read_data(output, 2, 2), 2u);
    EXPECT_EQ(output[0], 0x30);
    EXPECT_EQ(output[1], 0x40);
}

TEST_F(CircularBufferTest, BufferFullRejectsData)
{
    // Fill buffer to capacity (BUFFER_SIZE - 1 due to sentinel)
    std::vector<uint8_t> large_data(BUFFER_SIZE - 1);
    for (size_t i = 0; i < large_data.size(); ++i)
    {
        large_data[i] = static_cast<uint8_t>(i);
    }

    EXPECT_EQ(buffer.push_data(large_data.data(), static_cast<uint32_t>(large_data.size())),
              static_cast<uint32_t>(large_data.size()));
    EXPECT_EQ(buffer.remain(), 0u);

    // Additional push should fail (return 0)
    uint8_t extra = 0xFF;
    EXPECT_EQ(buffer.push_data(&extra, 1), 0u);
}

TEST_F(CircularBufferTest, WrapAround)
{
    uint8_t data1[200];
    uint8_t data2[100];
    uint8_t output[100];
    (void)output;

    for (int i = 0; i < 200; ++i)
        data1[i] = static_cast<uint8_t>(i);
    for (int i = 0; i < 100; ++i)
        data2[i] = static_cast<uint8_t>(i + 100);

    // Push 200 bytes
    buffer.push_data(data1, 200);

    // Pop 150 bytes
    buffer.pop_data(150);

    // Push 100 more bytes (should wrap around)
    buffer.push_data(data2, 100);

    // Read last 100 bytes
    uint8_t remaining[150];
    EXPECT_EQ(buffer.read_data(remaining, 150), 150u);

    // First 50 should be from data1[150..199]
    for (int i = 0; i < 50; ++i)
    {
        EXPECT_EQ(remaining[i], data1[150 + i]);
    }

    // Next 100 should be from data2
    for (int i = 0; i < 100; ++i)
    {
        EXPECT_EQ(remaining[50 + i], data2[i]);
    }
}

// ============================================================================
// Frame Header Tests
// ============================================================================

class FrameHeaderTest : public ::testing::Test
{
protected:
    unify_link_frame_head_t header;

    void SetUp() override { memset(&header, 0, sizeof(header)); }
};

TEST_F(FrameHeaderTest, SizeIs8Bytes)
{
    EXPECT_EQ(sizeof(unify_link_frame_head_t), 8u);
}

TEST_F(FrameHeaderTest, LengthAccessors)
{
    header.set_length(0x1234);
    EXPECT_EQ(header.length(), 0x1234 & 0x1FFF); // 13 bits max

    header.set_length(0x1FFF); // Max 13-bit value
    EXPECT_EQ(header.length(), 0x1FFF);

    header.set_length(0x2000); // Overflow, should mask
    EXPECT_EQ(header.length(), 0x0000);
}

TEST_F(FrameHeaderTest, FlagsAccessors)
{
    header.set_flags(0x05);
    EXPECT_EQ(header.flags(), 0x05);

    header.set_flags(0x07); // Max 3-bit value
    EXPECT_EQ(header.flags(), 0x07);

    // Verify length is preserved
    header.set_length(100);
    header.set_flags(0x03);
    EXPECT_EQ(header.length(), 100);
    EXPECT_EQ(header.flags(), 0x03);
}

TEST_F(FrameHeaderTest, CombinedFlagsAndLength)
{
    header.set_flags_and_length(0x05, 0x0ABC);
    EXPECT_EQ(header.flags(), 0x05);
    EXPECT_EQ(header.length(), 0x0ABC);
}

// ============================================================================
// Unify Link Base Tests
// ============================================================================

class UnifyLinkBaseTest : public ::testing::Test
{
protected:
    static constexpr uint32_t BUFFER_SIZE = 4096;
    Unify_link_base link;
};

TEST_F(UnifyLinkBaseTest, InitialCounters)
{
    EXPECT_EQ(link.success_count, 0u);
    EXPECT_EQ(link.com_error_count, 0u);
    EXPECT_EQ(link.decode_error_count, 0u);
}

TEST_F(UnifyLinkBaseTest, BuildAndParseFrame)
{
    // Register a handler for test data
    uint8_t received_data[64] = {0};
    link.register_handle_data(0x01, 0x02, received_data, nullptr, sizeof(received_data));

    // Build a frame
    uint8_t payload[64];
    for (int i = 0; i < 64; ++i)
        payload[i] = static_cast<uint8_t>(i);

    link.build_send_data(0x01, 0x02, payload, 64);

    // Get the built frame
    uint8_t frame_buffer[256];
    uint32_t frame_len = 0;
    link.send_buff_pop(frame_buffer, &frame_len);

    EXPECT_GT(frame_len, 0u);
    EXPECT_EQ(frame_buffer[0], FRAME_HEADER); // Frame header check

    // Push the frame to receive buffer and parse
    link.rev_data_push(frame_buffer, frame_len);
    link.parse_data_task();

    EXPECT_EQ(link.success_count, 1u);

    // Verify received data matches sent data
    EXPECT_EQ(memcmp(received_data, payload, 64), 0);
}

TEST_F(UnifyLinkBaseTest, SequenceIdIncrement)
{
    uint8_t dummy_data[16] = {0};
    link.register_handle_data(0x01, 0x01, dummy_data, nullptr, sizeof(dummy_data));

    // Send and receive multiple frames
    for (int i = 0; i < 5; ++i)
    {
        link.build_send_data(0x01, 0x01, dummy_data, 16);

        uint8_t frame[128];
        uint32_t len = 0;
        link.send_buff_pop(frame, &len);

        link.rev_data_push(frame, len);
        link.parse_data_task();
    }

    EXPECT_EQ(link.success_count, 5u);
}

TEST_F(UnifyLinkBaseTest, InvalidFrameHeader)
{
    uint8_t garbage[] = {0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
    link.rev_data_push(garbage, 5);
    link.parse_data_task();

    EXPECT_EQ(link.success_count, 0u);
}

TEST_F(UnifyLinkBaseTest, TransmissionErrorCrcCorruptionIncrementsNoSuccess)
{
    // Build a valid frame first.
    constexpr uint16_t payload_len = 8;
    uint8_t received[payload_len] = {0};
    link.register_handle_data(0x01, 0x02, received, nullptr, payload_len);

    uint8_t payload[payload_len] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    link.build_send_data(0x01, 0x02, payload, payload_len);

    uint8_t frame[128] = {0};
    uint32_t len = 0;
    link.send_buff_pop(frame, &len);
    ASSERT_GE(len, static_cast<uint32_t>(sizeof(unify_link_frame_head_t) + payload_len));

    // Corrupt CRC16 to simulate a transmission error (bit flip on the wire).
    // CRC16 is the last 2 bytes in unify_link_frame_head_t.
    frame[offsetof(unify_link_frame_head_t, crc16)] ^= 0xFF;

    const uint64_t success_before = link.success_count;
    const uint64_t decode_err_before = link.decode_error_count;
    const uint64_t com_err_before = link.com_error_count;

    link.rev_data_push(frame, len);
    link.parse_data_task();

    // CRC mismatch frames are dropped internally (by sliding 1 byte and re-syncing),
    // so they should not count as success and should not call handle_data().
    EXPECT_EQ(link.success_count, success_before);
    EXPECT_EQ(link.decode_error_count, decode_err_before);

    // Sequence error counter should not change either because CRC-failed frames never reach seq check.
    EXPECT_EQ(link.com_error_count, com_err_before);
}

// ============================================================================
// Integration Tests
// ============================================================================

class IntegrationTest : public ::testing::Test
{
protected:
    static constexpr uint32_t BUFFER_SIZE = 4096;
    Unify_link_base link_base;
};

TEST_F(IntegrationTest, MotorLinkRoundTrip)
{
    Motor_link_t motor_link(link_base);

    // Create motor info - 发送单个 motor_info_t
    Motor_link_t::motor_info_t sent_info = {.motor_id = 1,
                                            .ratio = 3.5f,
                                            .max_speed = 3000.0f,
                                            .max_current = 10.0f,
                                            .torque_constant = 0.1f,
                                            .max_position = 100000,
                                            .run_time = 500,
                                            .model = {"TestMotor"},
                                            .serial = {0x01, 0x02, 0x03},
                                            .firmware_version = 0x010203};

    // 发送单个 motor_info（handle_motor_info 会根据 motor_id 放到正确位置）
    link_base.build_send_data(Motor_link_t::component_id, Motor_link_t::MOTOR_INFO_ID,
                              reinterpret_cast<const uint8_t *>(&sent_info), sizeof(sent_info));

    // Get the frame
    uint8_t frame[256];
    uint32_t len = 0;
    link_base.send_buff_pop(frame, &len);

    // Receive and parse
    link_base.rev_data_push(frame, len);
    link_base.parse_data_task();

    EXPECT_EQ(link_base.success_count, 1u);

    // Verify the received data - motor_id=1，所以数据在 motor_info[1]
    EXPECT_EQ(motor_link.motor_info[1].motor_id, sent_info.motor_id);
    EXPECT_FLOAT_EQ(motor_link.motor_info[1].ratio, sent_info.ratio);
    EXPECT_FLOAT_EQ(motor_link.motor_info[1].max_speed, sent_info.max_speed);
}

TEST_F(IntegrationTest, EncoderLinkRoundTrip)
{
    Encoder_link_t encoder_link(link_base);

    // Create encoder info
    Encoder_link_t::encoder_info_t sent_info = {.encoder_id = 2,
                                                .resolution = 14,
                                                .max_velocity = 50000,
                                                .max_position = 16384,
                                                .run_time = 1000,
                                                .model = {"AS5047P"},
                                                .serial = {0xAA, 0xBB, 0xCC},
                                                .firmware_version = 0x020100};

    encoder_link.send_encoder_info_data(sent_info);

    uint8_t frame[256];
    uint32_t len = 0;
    link_base.send_buff_pop(frame, &len);

    link_base.rev_data_push(frame, len);
    link_base.parse_data_task();

    EXPECT_EQ(link_base.success_count, 1u);
    EXPECT_EQ(encoder_link.encoder_info.encoder_id, sent_info.encoder_id);
    EXPECT_EQ(encoder_link.encoder_info.resolution, sent_info.resolution);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
