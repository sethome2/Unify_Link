/**
 * @file motor_link_test.cpp
 * @brief Unit tests for Motor_link component
 */

#include "motor_link.hpp"
#include "unify_link.hpp"

#include <gtest/gtest.h>

using namespace unify_link;

class MotorLinkTest : public ::testing::Test
{
protected:
    static constexpr uint32_t BUFFER_SIZE = 4096;
    Unify_link_base link_base;
    Motor_link_t *motor_link;

    void SetUp() override { motor_link = new Motor_link_t(link_base); }

    void TearDown() override { delete motor_link; }

    // Helper to send and receive data through the link
    void roundTrip()
    {
        uint8_t frame[512];
        uint32_t len = 0;
        link_base.send_buff_pop(frame, &len);
        link_base.rev_data_push(frame, len);
        link_base.parse_data_task();
    }
};

TEST_F(MotorLinkTest, ComponentId)
{
    EXPECT_EQ(Motor_link_t::component_id, COMPONENT_ID_MOTORS);
}

TEST_F(MotorLinkTest, DataIds)
{
    EXPECT_EQ(Motor_link_t::MOTOR_BASIC_ID, 1);
    EXPECT_EQ(Motor_link_t::MOTOR_INFO_ID, 2);
    EXPECT_EQ(Motor_link_t::MOTOR_SETTING_ID, 3);
    EXPECT_EQ(Motor_link_t::MOTOR_SET_CURRENT_ID, 4);
}

TEST_F(MotorLinkTest, MotorBasicStruct)
{
    // Verify struct size is as expected (packed)
    Motor_link_t::motor_basic_t basic;
    EXPECT_EQ(sizeof(basic), 8u); // 2+2+2+1+1 = 8 bytes
}

TEST_F(MotorLinkTest, MotorInfoRoundTrip)
{
    // 发送单个 motor_info_t，handle_motor_info 会根据 motor_id 放到正确的数组位置
    Motor_link_t::motor_info_t sent = {
        .motor_id = 5,
        .ratio = 6.0f,
        .max_speed = 5000.0f,
        .max_current = 15.0f,
        .torque_constant = 0.05f,
        .max_position = 200000,
        .run_time = 1000,
        .model = {"TestMotor123"},
        .serial = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC},
        .firmware_version = 0x01020304};

    // 发送单个 motor_info
    link_base.build_send_data(Motor_link_t::component_id, Motor_link_t::MOTOR_INFO_ID,
                              reinterpret_cast<const uint8_t *>(&sent), sizeof(sent));
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    // motor_id=5，所以数据应该在 motor_info[5]
    EXPECT_EQ(motor_link->motor_info[5].motor_id, sent.motor_id);
    EXPECT_FLOAT_EQ(motor_link->motor_info[5].ratio, sent.ratio);
    EXPECT_FLOAT_EQ(motor_link->motor_info[5].max_speed, sent.max_speed);
    EXPECT_FLOAT_EQ(motor_link->motor_info[5].max_current, sent.max_current);
    EXPECT_FLOAT_EQ(motor_link->motor_info[5].torque_constant, sent.torque_constant);
    EXPECT_EQ(motor_link->motor_info[5].max_position, sent.max_position);
    EXPECT_EQ(motor_link->motor_info[5].run_time, sent.run_time);
    EXPECT_EQ(motor_link->motor_info[5].firmware_version, sent.firmware_version);
}

TEST_F(MotorLinkTest, MotorSettingRoundTrip)
{
    Motor_link_t::motor_settings_t sent = {.feedback_interval = 10, .reset_id = 3};

    motor_link->send_motor_setting_data(sent);
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    EXPECT_EQ(motor_link->motor_settings.feedback_interval, sent.feedback_interval);
    EXPECT_EQ(motor_link->motor_settings.reset_id, sent.reset_id);
}

TEST_F(MotorLinkTest, MotorSetCurrentRoundTrip)
{
    Motor_link_t::motor_set_t sent[Motor_link_t::MAX_MOTORS];
    for (int i = 0; i < Motor_link_t::MAX_MOTORS; ++i)
    {
        sent[i].motor_set = static_cast<int16_t>(1000 + i * 100);
        sent[i].motor_set_extra = 0;
    }

    motor_link->send_motor_set_data(sent);
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    for (int i = 0; i < Motor_link_t::MAX_MOTORS; ++i)
    {
        EXPECT_EQ(motor_link->motor_set[i].motor_set, sent[i].motor_set);
    }
}

TEST_F(MotorLinkTest, MultipleFrames)
{
    // 发送多个单独的 motor_info_t，每个都会根据 motor_id 放到正确位置
    for (int i = 0; i < 8; ++i) // 只测试有效的 motor_id (0-7)
    {
        Motor_link_t::motor_info_t info = {.motor_id = static_cast<uint8_t>(i)};
        link_base.build_send_data(Motor_link_t::component_id, Motor_link_t::MOTOR_INFO_ID,
                                  reinterpret_cast<const uint8_t *>(&info), sizeof(info));
        roundTrip();
        EXPECT_EQ(motor_link->motor_info[i].motor_id, static_cast<uint8_t>(i));
    }

    EXPECT_EQ(link_base.success_count, 8u);
}

TEST_F(MotorLinkTest, ErrorCodes)
{
    EXPECT_EQ(static_cast<uint8_t>(Motor_link_t::ErrorCode::OK), 0);
    EXPECT_EQ(static_cast<uint8_t>(Motor_link_t::ErrorCode::OVER_HEAT_ERR), 1);
    EXPECT_EQ(static_cast<uint8_t>(Motor_link_t::ErrorCode::INTERNAL_ERR), 255);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
