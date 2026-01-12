#pragma once

#ifndef __ENCODER_LINK_HPP__
#define __ENCODER_LINK_HPP__

#include "unify_link.hpp"

namespace unify_link
{
    class Encoder_link_t
    {
    public:
        enum class ErrorCode : uint8_t
        {
            OK = 0,
            OVERFLOW_ERR = 1,
            MAGNET_TOO_STRONG = 2,
            MAGNET_TOO_WEAK = 3,
            INTERNAL_ERR = 255
        };
#pragma pack(push, 1)
        constexpr static uint8_t ENCODER_BASIC_ID = 1;
        typedef struct
        {
            uint16_t position;    // 编码器位置
            int32_t velocity;     // 编码器速度
            ErrorCode error_code; // 错误码
        } encoder_basic_t;

        constexpr static uint8_t ENCODER_INFO_ID = 2;
        typedef struct
        {
            uint8_t encoder_id; // 编码器ID

            uint8_t resolution;        // 分辨率
            uint32_t max_velocity;     // 最大速度
            uint32_t max_position;     // 最大位置
            uint32_t run_time;         // 总运行时间
            char model[32];            // 编码器型号
            uint8_t serial[12];        // 编码器序列号
            uint32_t firmware_version; // 固件版本
        } encoder_info_t;

        constexpr static uint8_t ENCODER_SETTING_ID = 3;
        typedef struct
        {
            uint8_t feedback_interval; // 反馈间隔 ms
            uint8_t reset_id;          // 重设ID
        } encoder_setting_t;
#pragma pack(pop)

        constexpr static uint16_t MAX_ENCODERS = 8;
        encoder_basic_t encoder_basic[MAX_ENCODERS];
        encoder_info_t encoder_info;
        encoder_setting_t encoder_setting;

    public:
        Unify_link_base &link_base;
        constexpr static uint8_t component_id = COMPONENT_ID_ENCODERS; // 组件ID（与 motor_link 区分）

        Encoder_link_t(Unify_link_base &link_base) : link_base(link_base) { build_handle_data_matrix(); }
        ~Encoder_link_t() {};

        void build_handle_data_matrix()
        {
            link_base.register_handle_data(component_id, ENCODER_BASIC_ID, &encoder_basic, nullptr,
                                           sizeof(encoder_basic));
            link_base.register_handle_data(component_id, ENCODER_INFO_ID, &encoder_info, nullptr, sizeof(encoder_info));
            link_base.register_handle_data(component_id, ENCODER_SETTING_ID, &encoder_setting, nullptr,
                                           sizeof(encoder_setting));
        }

    public:
        void send_encoder_basic_data() { send_encoder_basic_data(encoder_basic); }
        void send_encoder_basic_data(const encoder_basic_t (&send_data)[MAX_ENCODERS])
        {
            link_base.send_packet<component_id>(ENCODER_BASIC_ID, send_data);
        }

        void send_encoder_info_data() { send_encoder_info_data(encoder_info); }
        void send_encoder_info_data(const encoder_info_t &send_data)
        {
            link_base.send_packet<component_id>(ENCODER_INFO_ID, send_data);
        }

        void send_encoder_setting_data() { send_encoder_setting_data(encoder_setting); }
        void send_encoder_setting_data(const encoder_setting_t &send_data)
        {
            link_base.send_packet<component_id>(ENCODER_SETTING_ID, send_data);
        }
    };
} // namespace unify_link

#endif