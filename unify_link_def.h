#ifndef UNIFY_LINK_H
#define UNIFY_LINK_H

#include <functional>
#include <stdbool.h>
#include <stdint.h>

// 组件ID定义
#define COMPONENT_ID_SYSTEM 0x00
#define COMPONENT_ID_MOTORS 0x01
#define COMPONENT_ID_UPDATE 0x02
#define COMPONENT_ID_ENCODERS 0x03
#define COMPONENT_ID_EXAMPLES 0x04

// 协议常量定义
#define FRAME_HEADER 0xA0
#define MAX_FRAME_DATA_LENGTH 512
#define MAX_FRAME_LENGTH (MAX_FRAME_DATA_LENGTH + sizeof(unify_link_frame_head_t))
#define MAX_RECV_BUFF_LENGTH (MAX_FRAME_DATA_LENGTH * 4)
#define MAX_SEND_BUFF_LENGTH (MAX_FRAME_DATA_LENGTH * 4)
#ifndef UNIFY_LINK_MAX_HANDLERS
#define UNIFY_LINK_MAX_HANDLERS 128
#endif

namespace unify_link
{
    struct unify_link_frame_head_t
    {
        uint8_t frame_header; // 帧头
        uint8_t seq_id;       // 序列号
        uint8_t component_id; // 组件ID
        uint8_t data_id;      // 数据ID
        // 高 3bit: flags, 低 13bit: length
        uint16_t payload_length_and_sign; // (3bits flags + 13 bits length)
        uint16_t crc16;                   // CRC-16校验码

        static constexpr uint16_t kLenMask = 0x1FFF;  // 13 bits
        static constexpr uint16_t kFlagMask = 0xE000; // 3 bits (bit15..13)
        static constexpr uint8_t kFlagShift = 13;

        // 读取/设置 13bit 长度
        inline uint16_t length() const { return static_cast<uint16_t>(payload_length_and_sign & kLenMask); }
        inline void set_length(uint16_t len)
        {
            payload_length_and_sign = static_cast<uint16_t>((payload_length_and_sign & kFlagMask) | (len & kLenMask));
        }

        // 读取/设置 3bit flags
        inline uint8_t flags() const { return static_cast<uint8_t>((payload_length_and_sign >> kFlagShift) & 0x7); }
        inline void set_flags(uint8_t flags)
        {
            payload_length_and_sign = static_cast<uint16_t>((payload_length_and_sign & kLenMask) |
                                                            ((static_cast<uint16_t>(flags) & 0x7) << kFlagShift));
        }

        // 一次性打包
        inline void set_flags_and_length(uint8_t flags, uint16_t len)
        {
            payload_length_and_sign =
                static_cast<uint16_t>(((static_cast<uint16_t>(flags) & 0x7) << kFlagShift) | (len & kLenMask));
        }
    };

    static_assert(sizeof(unify_link_frame_head_t) == 8, "unify_link_frame_head_t must be 8 bytes");

    // 回调函数类型定义：处理数据载荷，返回是否成功
    // 参数：数据指针、长度
    // 返回值：true 表示处理成功，false 表示失败
    // 使用 std::function 支持普通函数、lambda、成员函数绑定
    using handle_data_func_t = std::function<bool(const uint8_t *, uint16_t)>;
    struct registered_item_t
    {
        handle_data_func_t callback;
        void *dst = nullptr;
        uint16_t payload_length = 0;
    };

    // todo: add error codes into Unify_link_base
    enum class Error_code_e
    {
        NONE = 0,
        COM_ERROR,
        DECODE_ERROR,
    };

} // namespace unify_link

#endif // UNIFY_LINK_H
