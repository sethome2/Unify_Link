#pragma once

#ifndef __MOTOR_LINK_HPP__
#define __MOTOR_LINK_HPP__

#include "unify_link.hpp"

#include "unify_link_def.h"

namespace unify_link
{
    class Motor_link_t
    {
    public:
        enum class ErrorCode : uint8_t
        {
            OK = 0,
            OVER_HEAT_ERR = 1,
            INTERNAL_ERR = 255
        };

        enum class MotorMode : uint8_t
        {
            CURRENT_CONTROL = 0,
            SPEED_CONTROL = 1,
            POSITION_CONTROL = 2,
            MIT_CONTROL = 3
        };

#pragma pack(push, 1)
        constexpr static uint8_t MOTOR_BASIC_ID = 1;
        typedef struct
        {
            uint16_t position;  // 位置
            int16_t speed;      // 速度
            uint16_t current;   // 电流
            int8_t temperature; // 温度

            ErrorCode error_code; // 错误码
        } motor_basic_t;

        constexpr static uint8_t MOTOR_INFO_ID = 2;
        typedef struct
        {
            uint8_t motor_id;

            float ratio;           // 减速比
            float max_speed;       // 最大速度 rad/s
            float max_current;     // 最大电流 A
            float torque_constant; // 扭矩常数 Nm/A
            uint32_t max_position; // 最大位置
            uint32_t run_time;     // 总运行时间 Hours

            char model[32];            // 电机型号
            uint8_t serial[12];        // 电机序列号 96bit
            uint32_t firmware_version; // 固件版本
        } motor_info_t;

        constexpr static uint8_t MOTOR_SETTING_ID = 3;
        typedef struct
        {
            uint8_t feedback_interval; // 反馈间隔 ms
            uint8_t reset_id;          // 重设ID
            MotorMode mode;            // 电机模式
        } motor_settings_t;

        constexpr static uint8_t MOTOR_SET_CURRENT_ID = 4;
        typedef struct
        {
            int16_t motor_set;
            int16_t motor_set_extra;
        } motor_set_t;
#pragma pack(pop)

        // Maximum number of motors
        constexpr static uint8_t MAX_MOTORS = 8;
        motor_basic_t motor_basic[MAX_MOTORS];
        motor_info_t motor_info[MAX_MOTORS];
        motor_settings_t motor_settings;
        motor_set_t motor_set[MAX_MOTORS];

    public:
        Unify_link_base &link_base;
        constexpr static uint8_t component_id = COMPONENT_ID_MOTORS; // 组件ID

        Motor_link_t(Unify_link_base &link_base) : link_base(link_base) { build_handle_data_matrix(); }
        ~Motor_link_t() {};

        void build_handle_data_matrix()
        {
            // 注册数据处理函数
            link_base.register_handle_data(component_id, MOTOR_BASIC_ID, &motor_basic, nullptr, sizeof(motor_basic));

            link_base.register_handle_data(
                component_id, MOTOR_INFO_ID, &motor_info, [this](const uint8_t *data, uint16_t len)
                { return this->handle_motor_info(data, len); }, sizeof(motor_info_t));

            link_base.register_handle_data(component_id, MOTOR_SETTING_ID, &motor_settings, nullptr,
                                           sizeof(motor_settings));

            link_base.register_handle_data(component_id, MOTOR_SET_CURRENT_ID, &motor_set, nullptr, sizeof(motor_set));
        }

    public:
        bool handle_motor_info(const uint8_t *data, uint16_t len)
        {
            (void)len;

            motor_info_t *info = reinterpret_cast<motor_info_t *>(const_cast<uint8_t *>(data));

            if (info->motor_id < MAX_MOTORS)
            {
                memcpy(&motor_info[info->motor_id], info, sizeof(motor_info_t));

                return true;
            }

            return false;
        }

        void send_motor_basic_data() { send_motor_basic_data(motor_basic); }
        void send_motor_basic_data(const motor_basic_t (&send_data)[MAX_MOTORS])
        {
            link_base.send_packet<component_id>(MOTOR_BASIC_ID, send_data);
        }

        void send_motor_info_data() { send_motor_info_data(motor_info); }
        void send_motor_info_data(const motor_info_t (&send_data)[MAX_MOTORS])
        {
            link_base.send_packet<component_id>(MOTOR_INFO_ID, send_data);
        }

        void send_motor_setting_data() { send_motor_setting_data(motor_settings); }
        void send_motor_setting_data(const motor_settings_t &send_data)
        {
            link_base.send_packet<component_id>(MOTOR_SETTING_ID, send_data);
        }

        void send_motor_set_data() { send_motor_set_data(motor_set); }
        void send_motor_set_data(const motor_set_t (&send_data)[MAX_MOTORS])
        {
            link_base.send_packet<component_id>(MOTOR_SET_CURRENT_ID, send_data);
        }
        void set_motor_current(uint8_t motor_id, int16_t current)
        {
            if (motor_id >= MAX_MOTORS)
                return;

            motor_set[motor_id].motor_set = current;
            motor_set[motor_id].motor_set_extra = 0;
        }
    };
} // namespace unify_link

#endif