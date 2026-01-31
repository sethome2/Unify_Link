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
    EXPECT_EQ(Motor_link_t::MOTOR_SET_ID, 4);
}

TEST_F(MotorLinkTest, MotorBasicStruct)
{
    // Verify struct size is as expected (packed)
    Motor_link_t::feedback_t basic;
    EXPECT_EQ(sizeof(basic), 8u); // 2+2+2+1+1 = 8 bytes
}

TEST_F(MotorLinkTest, MotorInfoRoundTrip)
{
    // 发送单个 motor_info_t，handle_motor_info 会根据 motor_id 放到正确的数组位置
    Motor_link_t::info_t sent = {.motor_id = 5,
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
    Motor_link_t::settings_t sent = {
        .motor_id = 4, .feedback_interval = 10, .reset_id = 3, .mode = Motor_link_t::MotorMode::SPEED_CONTROL};

    link_base.build_send_data(Motor_link_t::component_id, Motor_link_t::MOTOR_SETTING_ID,
                              reinterpret_cast<const uint8_t *>(&sent), sizeof(sent));
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    EXPECT_EQ(motor_link->motor_settings[sent.motor_id].feedback_interval, sent.feedback_interval);
    EXPECT_EQ(motor_link->motor_settings[sent.motor_id].reset_id, sent.reset_id);
    EXPECT_EQ(motor_link->motor_settings[sent.motor_id].mode, sent.mode);
}

TEST_F(MotorLinkTest, MotorSetCurrentRoundTrip)
{
    Motor_link_t::set_t sent[Motor_link_t::MAX_MOTORS];
    for (int i = 0; i < Motor_link_t::MAX_MOTORS; ++i)
    {
        sent[i].set = static_cast<int16_t>(1000 + i * 100);
        sent[i].set_extra = 0;
        sent[i].set_extra2 = 0;
    }

    motor_link->send_motor_set_data(sent);
    roundTrip();

    EXPECT_EQ(link_base.success_count, 1u);
    for (int i = 0; i < Motor_link_t::MAX_MOTORS; ++i)
    {
        EXPECT_EQ(motor_link->motor_set[i].set, sent[i].set);
    }
}

TEST_F(MotorLinkTest, MotorSetSpeed)
{
    const uint8_t motor_id = 1;
    motor_link->motor_settings[motor_id].mode = Motor_link_t::MotorMode::SPEED_CONTROL;

    const int16_t speed = 1500;
    EXPECT_TRUE(motor_link->set_motor_speed(motor_id, speed));
    EXPECT_EQ(motor_link->motor_set[motor_id].set, speed);
    EXPECT_EQ(motor_link->motor_set[motor_id].set_extra, 0);
    EXPECT_EQ(motor_link->motor_set[motor_id].set_extra2, 0);
}

TEST_F(MotorLinkTest, MotorSetPosition)
{
    const uint8_t motor_id = 2;
    motor_link->motor_settings[motor_id].mode = Motor_link_t::MotorMode::POSITION_CONTROL;

    const uint16_t position = 3200;
    const int16_t speed = 120;
    EXPECT_TRUE(motor_link->set_motor_position(motor_id, position, speed));
    EXPECT_EQ(motor_link->motor_set[motor_id].set, static_cast<int16_t>(position));
    EXPECT_EQ(motor_link->motor_set[motor_id].set_extra, speed);
    EXPECT_EQ(motor_link->motor_set[motor_id].set_extra2, 0);
}

TEST_F(MotorLinkTest, MotorSetMit)
{
    const uint8_t motor_id = 3;
    motor_link->motor_settings[motor_id].mode = Motor_link_t::MotorMode::MIT_CONTROL;

    const uint16_t position = 2048;
    const int16_t speed = 200;
    const uint16_t current = 50;
    EXPECT_TRUE(motor_link->set_motor_mit(motor_id, position, speed, current));
    EXPECT_EQ(motor_link->motor_set[motor_id].set, static_cast<int16_t>(position));
    EXPECT_EQ(motor_link->motor_set[motor_id].set_extra, speed);
    EXPECT_EQ(motor_link->motor_set[motor_id].set_extra2, static_cast<int16_t>(current));
}

TEST_F(MotorLinkTest, MotorSetModeMismatchReturnsFalse)
{
    const uint8_t motor_id = 4;
    motor_link->motor_settings[motor_id].mode = Motor_link_t::MotorMode::CURRENT_CONTROL;

    EXPECT_FALSE(motor_link->set_motor_speed(motor_id, 100));
    EXPECT_FALSE(motor_link->set_motor_position(motor_id, 1000, 10));
    EXPECT_FALSE(motor_link->set_motor_mit(motor_id, 1000, 10, 5));

    motor_link->motor_settings[motor_id].mode = Motor_link_t::MotorMode::SPEED_CONTROL;
    EXPECT_FALSE(motor_link->set_motor_current(motor_id, 100));
}

TEST_F(MotorLinkTest, MotorInfoCallback)
{
    Motor_link_t::info_t callback_info{};
    bool called = false;
    motor_link->on_motor_info_updated = [&](const Motor_link_t::info_t &info)
    {
        callback_info = info;
        called = true;
    };

    Motor_link_t::info_t sent = {.motor_id = 2,
                                 .ratio = 3.2f,
                                 .max_speed = 1200.0f,
                                 .max_current = 8.0f,
                                 .torque_constant = 0.03f,
                                 .max_position = 4200,
                                 .run_time = 12,
                                 .model = {"CallbackMotor"},
                                 .serial = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C},
                                 .firmware_version = 0x0A0B0C0D};

    link_base.build_send_data(Motor_link_t::component_id, Motor_link_t::MOTOR_INFO_ID,
                              reinterpret_cast<const uint8_t *>(&sent), sizeof(sent));
    roundTrip();

    EXPECT_TRUE(called);
    EXPECT_EQ(callback_info.motor_id, sent.motor_id);
    EXPECT_FLOAT_EQ(callback_info.ratio, sent.ratio);
}

TEST_F(MotorLinkTest, MotorSettingsCallback)
{
    Motor_link_t::settings_t callback_settings{};
    bool called = false;
    motor_link->on_motor_settings_updated = [&](const Motor_link_t::settings_t &settings)
    {
        callback_settings = settings;
        called = true;
    };

    Motor_link_t::settings_t sent = {
        .motor_id = 6, .feedback_interval = 20, .reset_id = 2, .mode = Motor_link_t::MotorMode::POSITION_CONTROL};

    link_base.build_send_data(Motor_link_t::component_id, Motor_link_t::MOTOR_SETTING_ID,
                              reinterpret_cast<const uint8_t *>(&sent), sizeof(sent));
    roundTrip();

    EXPECT_TRUE(called);
    EXPECT_EQ(callback_settings.motor_id, sent.motor_id);
    EXPECT_EQ(callback_settings.feedback_interval, sent.feedback_interval);
    EXPECT_EQ(callback_settings.reset_id, sent.reset_id);
    EXPECT_EQ(callback_settings.mode, sent.mode);
}

TEST_F(MotorLinkTest, MultipleFrames)
{
    // 发送多个单独的 motor_info_t，每个都会根据 motor_id 放到正确位置
    for (int i = 0; i < 8; ++i) // 只测试有效的 motor_id (0-7)
    {
        Motor_link_t::info_t info = {.motor_id = static_cast<uint8_t>(i)};
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
