#ifndef __UPDATE_LINK_HPP__
#define __UPDATE_LINK_HPP__

#include "unify_link.hpp"

namespace unify_link
{
    class Update_Link_t
    {
    public:
        // 消息ID定义
        constexpr static uint8_t FIRMWARE_INFO_ID = 1;
        constexpr static uint8_t FIRMWARE_CRC_ID = 2;

#pragma pack(push, 1)
        typedef struct
        {
            uint8_t firmware_data[256]; // 修改为 256 字节以符合 MAX_FRAME_DATA_LENGTH 限制
        } firmware_info_t;

        typedef struct
        {
            uint16_t crc16;
        } firmware_crc_t;
#pragma pack(pop)

        firmware_info_t firmware_info;
        firmware_crc_t firmware_crc;

        Unify_link_base &link_base;

        constexpr static uint8_t component_id = COMPONENT_ID_UPDATE;

        Update_Link_t(Unify_link_base &link_base) : link_base(link_base) { build_handle_data_matrix(); }
        ~Update_Link_t() {}

        void build_handle_data_matrix()
        {
            link_base.register_handle_data(component_id, FIRMWARE_INFO_ID, &firmware_info, nullptr,
                                           sizeof(firmware_info));
            link_base.register_handle_data(component_id, FIRMWARE_CRC_ID, &firmware_crc, nullptr, sizeof(firmware_crc));
        }

        void send_firmware_info() { send_firmware_info(firmware_info); }
        void send_firmware_info(const firmware_info_t &info)
        {
            link_base.send_packet<component_id>(FIRMWARE_INFO_ID, info);
        }

        void send_firmware_crc() { send_firmware_crc(firmware_crc); }
        void send_firmware_crc(const firmware_crc_t &crc) { link_base.send_packet<component_id>(FIRMWARE_CRC_ID, crc); }
    };
} // namespace unify_link

#endif