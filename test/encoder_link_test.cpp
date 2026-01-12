/**
 * @file encoder_link_test.cpp
 * @brief Unit tests for Encoder_link component
 */

#include "encoder_link.hpp"
#include "unify_link.hpp"

#include <gtest/gtest.h>

using namespace unify_link;

class EncoderLinkTest : public ::testing::Test
{
protected:
    static constexpr uint32_t BUFFER_SIZE = 4096;
    Unify_link_base link_base;
    Encoder_link_t *encoder_link;

    void SetUp() override { encoder_link = new Encoder_link_t(link_base); }

    void TearDown() override { delete encoder_link; }

    void roundTrip()
    {
        uint8_t frame[512];
        uint32_t len = 0;
        link_base.send_buff_pop(frame, &len);
        link_base.rev_data_push(frame, len);
        link_base.parse_data_task();
    }
};

TEST_F(EncoderLinkTest, ComponentId)
{
    EXPECT_EQ(Encoder_link_t::component_id, COMPONENT_ID_ENCODERS);
}

TEST_F(EncoderLinkTest, DataIds)
{
    EXPECT_EQ(Encoder_link_t::ENCODER_BASIC_ID, 1);
    EXPECT_EQ(Encoder_link_t::ENCODER_INFO_ID, 2);
    EXPECT_EQ(Encoder_link_t::ENCODER_SETTING_ID, 3);
}

TEST_F(EncoderLinkTest, EncoderBasicStruct)
{
    Encoder_link_t::encoder_basic_t basic;
    EXPECT_EQ(sizeof(basic), 7u); // 2+4+1 = 7 bytes (with #pragma pack(1))
}

TEST_F(EncoderLinkTest, EncoderInfoRoundTrip)
{
    Encoder_link_t::encoder_info_t sent = {
        .encoder_id = 3,
        .resolution = 14,
        .max_velocity = 100000,
        .max_position = 16384,
        .run_time = 5000,
        .model = {"AS5048A"},
        .serial = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
        .firmware_version = 0x02010300};

    encoder_link->send_encoder_info_data(sent);
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    EXPECT_EQ(encoder_link->encoder_info.encoder_id, sent.encoder_id);
    EXPECT_EQ(encoder_link->encoder_info.resolution, sent.resolution);
    EXPECT_EQ(encoder_link->encoder_info.max_velocity, sent.max_velocity);
    EXPECT_EQ(encoder_link->encoder_info.max_position, sent.max_position);
    EXPECT_EQ(encoder_link->encoder_info.run_time, sent.run_time);
    EXPECT_EQ(encoder_link->encoder_info.firmware_version, sent.firmware_version);
}

TEST_F(EncoderLinkTest, EncoderSettingRoundTrip)
{
    Encoder_link_t::encoder_setting_t sent = {.feedback_interval = 20, .reset_id = 5};

    encoder_link->send_encoder_setting_data(sent);
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    EXPECT_EQ(encoder_link->encoder_setting.feedback_interval, sent.feedback_interval);
    EXPECT_EQ(encoder_link->encoder_setting.reset_id, sent.reset_id);
}

TEST_F(EncoderLinkTest, MultipleEncoders)
{
    for (int i = 0; i < Encoder_link_t::MAX_ENCODERS; ++i)
    {
        Encoder_link_t::encoder_info_t info = {.encoder_id = static_cast<uint8_t>(i)};
        encoder_link->send_encoder_info_data(info);
        roundTrip();
    }

    EXPECT_EQ(link_base.success_count, Encoder_link_t::MAX_ENCODERS);
}

TEST_F(EncoderLinkTest, ErrorCodes)
{
    EXPECT_EQ(static_cast<uint8_t>(Encoder_link_t::ErrorCode::OK), 0);
    EXPECT_EQ(static_cast<uint8_t>(Encoder_link_t::ErrorCode::OVERFLOW_ERR), 1);
    EXPECT_EQ(static_cast<uint8_t>(Encoder_link_t::ErrorCode::MAGNET_TOO_STRONG), 2);
    EXPECT_EQ(static_cast<uint8_t>(Encoder_link_t::ErrorCode::MAGNET_TOO_WEAK), 3);
    EXPECT_EQ(static_cast<uint8_t>(Encoder_link_t::ErrorCode::INTERNAL_ERR), 255);
}

TEST_F(EncoderLinkTest, BasicArraySize)
{
    EXPECT_EQ(Encoder_link_t::MAX_ENCODERS, 8u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
