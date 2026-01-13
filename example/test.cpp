#include "motor_link.hpp"
#include "unify_link.hpp"

#include <cassert>
#include <iomanip>
#include <iostream>
#include <vector>

unify_link::Unify_link_base unify_link_base;

unify_link::Motor_link_t motor_link(unify_link_base);

// motor_info_t 字段顺序应与 motor_link.hpp 声明一致
unify_link::Motor_link_t::motor_info_t motor_info = {
    0,              // motor_id
    1.0f,           // ratio
    3000.0f,        // max_speed (rad/s)
    10.0f,          // max_current
    0.1f,           // torque_constant
    1000000,        // max_position
    120,            // run_time
    {"Test Motor"}, // model
    {0},            // serial
    0               // firmware_version
};

unify_link::Motor_link_t::motor_info_t motor_info_arr[unify_link::Motor_link_t::MAX_MOTORS] = {};

uint8_t buff[4096] = {0};
uint32_t buff_len = 0;

int main()
{
    motor_link.on_motor_info_updated = [](const unify_link::Motor_link_t::motor_info_t &info)
    {
        std::cout << "Motor info updated. motor_id=" << static_cast<int>(info.motor_id) << " model=" << info.model
                  << std::endl;
    };

    for (int i = 0; i < 10; ++i)
    {
        motor_info_arr[0] = motor_info;
        unify_link_base.build_send_data(unify_link::Motor_link_t::component_id,
                                        unify_link::Motor_link_t::MOTOR_INFO_ID,
                                        reinterpret_cast<const uint8_t *>(&motor_info_arr[0]),
                                        sizeof(motor_info_arr[0]));
        unify_link_base.send_buff_pop(buff, &buff_len);
        unify_link_base.rev_data_push(buff, buff_len);
        unify_link_base.parse_data_task();
    }

    std::cout << "Success count: " << unify_link_base.success_count << std::endl;
    std::cout << "Communication error count: " << unify_link_base.com_error_count << std::endl;
    std::cout << "Decode error count: " << unify_link_base.decode_error_count << std::endl;
    std::cout << "Last sequence ID: " << static_cast<int>(unify_link_base.last_seq_id) << std::endl;

    return 0;
}
