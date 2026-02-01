#pragma once

#include <cstring>
#ifndef __MOTOR_LINK_HPP__
#define __MOTOR_LINK_HPP__

#include "unify_link.hpp"

#include "unify_link_def.h"

#include <functional>

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
        } feedback_t;

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
        } info_t;

        constexpr static uint8_t MOTOR_SETTING_ID = 3;
        typedef struct
        {
            uint8_t motor_id;

            uint8_t feedback_interval; // 反馈间隔 ms
            uint8_t reset_id;          // 重设ID
            MotorMode mode;            // 电机模式
        } settings_t;

        constexpr static uint8_t MOTOR_SET_ID = 4;
        typedef struct
        {
            int16_t set;
            int16_t set_extra;
            int16_t set_extra2;
        } set_t;

        constexpr static uint8_t MOTOR_PID_ID = 5;
        typedef struct
        {
            uint8_t motor_id;

            PIDParams_t current_pid;
            PIDParams_t speed_pid;
            PIDParams_t position_pid;
        } pid_t;

#pragma pack(pop)

        // Maximum number of motors
        constexpr static uint8_t MAX_MOTORS = 8;
        feedback_t motor_basic[MAX_MOTORS];
        info_t motor_info[MAX_MOTORS];
        settings_t motor_settings[MAX_MOTORS];
        set_t motor_set[MAX_MOTORS];
        pid_t motor_pid;

    public:
        std::function<void(const feedback_t (&)[MAX_MOTORS])> on_motor_basic_updated;
        std::function<void(const info_t &)> on_motor_info_updated;
        std::function<void(const settings_t &)> on_motor_settings_updated;
        std::function<void(const set_t (&)[MAX_MOTORS])> on_motor_set_updated;
        std::function<void(const pid_t &)> on_motor_pid_updated;

        Unify_link_base &link_base;
        constexpr static uint8_t component_id = COMPONENT_ID_MOTORS; // 组件ID

        Motor_link_t(Unify_link_base &link_base) : link_base(link_base) { build_handle_data_matrix(); }
        ~Motor_link_t() {};

        void build_handle_data_matrix()
        {
            // 注册数据处理函数
            link_base.register_handle_data(
                component_id, MOTOR_BASIC_ID, &motor_basic,
                [this](const uint8_t *data, uint16_t len) { return this->handle_motor_basic(data, len); },
                sizeof(motor_basic));

            link_base.register_handle_data(
                component_id, MOTOR_INFO_ID, nullptr, [this](const uint8_t *data, uint16_t len)
                { return this->handle_motor_info(data, len); }, sizeof(info_t));

            link_base.register_handle_data(
                component_id, MOTOR_SETTING_ID, nullptr, [this](const uint8_t *data, uint16_t len)
                { return this->handle_motor_settings(data, len); }, sizeof(settings_t));

            link_base.register_handle_data(
                component_id, MOTOR_SET_ID, nullptr,
                [this](const uint8_t *data, uint16_t len)
                {
                    if (len != sizeof(motor_set))
                    {
                        return false;
                    }

                    std::memcpy(motor_set, data, sizeof(motor_set));
                    if (on_motor_set_updated)
                    {
                        on_motor_set_updated(motor_set);
                    }
                    return true;
                },
                sizeof(motor_set));

            link_base.register_handle_data(
                component_id, MOTOR_PID_ID, &motor_pid,
                [this](const uint8_t *data, uint16_t len) { return this->handle_motor_pid(data, len); }, sizeof(pid_t));
        }

    public:
        template <typename T>
        bool handle_motor_payload(const uint8_t *data, uint16_t len, T (&target)[MAX_MOTORS],
                                  std::function<void(const T &)> &updated_cb)
        {
            (void)len;

            const T *payload = reinterpret_cast<const T *>(data);

            if (payload->motor_id < MAX_MOTORS)
            {
                memcpy(&target[payload->motor_id], payload, sizeof(T));
                if (updated_cb)
                {
                    updated_cb(target[payload->motor_id]);
                }
                return true;
            }

            return false;
        }

        bool handle_motor_basic(const uint8_t *data, uint16_t len)
        {
            // already copied by unify_link_base
            (void)len;
            (void)data;

            if (on_motor_basic_updated)
                on_motor_basic_updated(motor_basic);
            return true;
        }

        bool handle_motor_info(const uint8_t *data, uint16_t len)
        {
            return handle_motor_payload(data, len, motor_info, on_motor_info_updated);
        }

        bool handle_motor_settings(const uint8_t *data, uint16_t len)
        {
            return handle_motor_payload(data, len, motor_settings, on_motor_settings_updated);
        }

        bool handle_motor_pid(const uint8_t *data, uint16_t len)
        {
            // already copied by unify_link_base
            (void)len;
            (void)data;

            if (on_motor_pid_updated)
            {
                on_motor_pid_updated(motor_pid);
            }
            return true;
        }

        void send_motor_basic_data() { send_motor_basic_data(motor_basic); }
        void send_motor_basic_data(const feedback_t (&send_data)[MAX_MOTORS])
        {
            link_base.send_packet<component_id>(MOTOR_BASIC_ID, send_data);
        }

        void send_motor_info_data(uint8_t motor_id)
        {
            if (motor_id >= MAX_MOTORS)
                return;

            send_motor_info_data(motor_info[motor_id]);
        }
        void send_motor_info_data(const info_t &send_data)
        {
            link_base.send_packet<component_id>(MOTOR_INFO_ID, send_data);
        }

        void send_motor_setting_data(uint8_t motor_id)
        {
            if (motor_id >= MAX_MOTORS)
                return;

            send_motor_setting_data(motor_settings[motor_id]);
        }
        void send_motor_setting_data(const settings_t &send_data)
        {
            link_base.send_packet<component_id>(MOTOR_SETTING_ID, send_data);
        }

        void send_motor_set_data() { send_motor_set_data(motor_set); }
        void send_motor_set_data(const set_t (&send_data)[MAX_MOTORS])
        {
            link_base.send_packet<component_id>(MOTOR_SET_ID, send_data);
        }

        bool set_motor_mode(uint8_t motor_id, MotorMode mode)
        {
            if (motor_id >= MAX_MOTORS)
                return false;

            motor_settings[motor_id].mode = mode;
            send_motor_setting_data(motor_settings[motor_id]);
            return true;
        }

        bool set_motor_current(uint8_t motor_id, int16_t currentq, int16_t currentd = 0)
        {
            if (motor_id >= MAX_MOTORS)
                return false;

            if (motor_settings[motor_id].mode != MotorMode::CURRENT_CONTROL)
                return false;

            motor_set[motor_id].set = currentq;
            motor_set[motor_id].set_extra = currentd;
            motor_set[motor_id].set_extra2 = 0;
            return true;
        }

        bool set_motor_speed(uint8_t motor_id, int16_t speed)
        {
            if (motor_id >= MAX_MOTORS)
                return false;

            if (motor_settings[motor_id].mode != MotorMode::SPEED_CONTROL)
                return false;

            motor_set[motor_id].set = speed;
            motor_set[motor_id].set_extra = 0;
            motor_set[motor_id].set_extra2 = 0;
            return true;
        }

        bool set_motor_position(uint8_t motor_id, uint16_t position, int16_t speed = 0)
        {
            if (motor_id >= MAX_MOTORS)
                return false;

            if (motor_settings[motor_id].mode != MotorMode::POSITION_CONTROL)
                return false;

            motor_set[motor_id].set = static_cast<int16_t>(position);
            motor_set[motor_id].set_extra = speed;
            motor_set[motor_id].set_extra2 = 0;
            return true;
        }

        bool set_motor_mit(uint8_t motor_id, uint16_t position, int16_t speed = 0, uint16_t current = 0)
        {
            if (motor_id >= MAX_MOTORS)
                return false;

            if (motor_settings[motor_id].mode != MotorMode::MIT_CONTROL)
                return false;

            motor_set[motor_id].set = static_cast<int16_t>(position);
            motor_set[motor_id].set_extra = speed;
            motor_set[motor_id].set_extra2 = current;
            return true;
        }
    };
} // namespace unify_link

#endif
