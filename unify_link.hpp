#ifndef UNIFY_LINK_HPP
#define UNIFY_LINK_HPP

#include "CRC16.hpp"

#include "unify_link_def.h"

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>

namespace unify_link
{
    using namespace std;

    template <typename T, uint32_t N>
    class Circular_buffer
    {
    public:
        std::array<T, N> buf{}; // 缓冲区
        // SPSC (single producer / single consumer) ring buffer.
        // 约定：
        //   - 生产者（例如：串口中断）仅调用 push_data()
        //   - 消费者（例如：主循环）仅调用 read_data()/pop_data()/used()/remain()
        // 这样可在不加锁的情况下实现线程/中断安全。
        std::atomic<uint32_t> head{0};
        std::atomic<uint32_t> tail{0}; // 预留 1 字节哨兵

        uint32_t used() const
        {
            // consumer-side safe (also safe if producer reads used()).
            uint32_t h = head.load(std::memory_order_acquire);
            uint32_t t = tail.load(std::memory_order_acquire);
            return (h + N - t) % N;
        }

        uint32_t remain() const { return N - 1 - used(); }

    public:
        Circular_buffer()
        {
            buf.fill(0);
            head.store(0, std::memory_order_relaxed);
            tail.store(0, std::memory_order_relaxed);
        }
        virtual ~Circular_buffer() {}

        // 拷贝和移动构造函数
        Circular_buffer(const Circular_buffer &other)
        {
            buf = other.buf;
            head.store(other.head.load(std::memory_order_relaxed), std::memory_order_relaxed);
            tail.store(other.tail.load(std::memory_order_relaxed), std::memory_order_relaxed);
        }

        Circular_buffer(Circular_buffer &&other) noexcept
        {
            buf = std::move(other.buf);
            head.store(other.head.load(std::memory_order_relaxed), std::memory_order_relaxed);
            tail.store(other.tail.load(std::memory_order_relaxed), std::memory_order_relaxed);
            other.head.store(0, std::memory_order_relaxed);
            other.tail.store(0, std::memory_order_relaxed);
        }

        uint32_t push_data(const T *src, uint32_t len)
        {
            // producer-only
            if (len == 0)
                return 0;

            uint32_t h = head.load(std::memory_order_relaxed);
            uint32_t t = tail.load(std::memory_order_acquire);

            uint32_t used_local = (h + N - t) % N;
            uint32_t free_local = N - 1 - used_local;
            if (len > free_local)
                return 0;

            uint32_t first = std::min<uint32_t>(len, N - h);
            std::memcpy(buf.data() + h, src, first);
            std::memcpy(buf.data(), src + first, len - first);

            std::atomic_thread_fence(std::memory_order_release);
            head.store((h + len) % N, std::memory_order_release);
            return len;
        }

        uint32_t read_data(T *dst, uint32_t len) { return read_data(dst, len, 0); }

        uint32_t read_data(T *dst, uint32_t len, uint32_t offset) const
        {
            // consumer-only (read without popping)
            if (len == 0)
                return 0;

            uint32_t t = tail.load(std::memory_order_relaxed);
            uint32_t h = head.load(std::memory_order_acquire);
            uint32_t used_local = (h + N - t) % N;
            if (offset + len > used_local)
                return 0;

            uint32_t start = (t + offset) % N;

            uint32_t first = std::min<uint32_t>(len, N - start);
            std::memcpy(dst, buf.data() + start, first);
            std::memcpy(dst + first, buf.data(), len - first);

            return len; // 不移动 tail
        }

        // 从尾部弹出数据
        uint32_t pop_data(uint32_t len)
        {
            // consumer-only
            if (len == 0)
                return 0;

            uint32_t t = tail.load(std::memory_order_relaxed);
            uint32_t h = head.load(std::memory_order_acquire);
            uint32_t used_local = (h + N - t) % N;
            if (len > used_local)
                return 0;

            tail.store((t + len) % N, std::memory_order_release);
            return len;
        }
    };

    class Unify_link_base
    {
    protected:
        bool _find_frame_head()
        {
            while (rec_buff.used() >= sizeof(frame_head))
            {
                rec_buff.read_data(reinterpret_cast<uint8_t *>(&frame_head), sizeof(frame_head));
                if (frame_head.frame_header == FRAME_HEADER)
                    return true;

                rec_buff.pop_data(1); // 向后滑动 1 字节
            }
            return false; // 数据不足
        }

    public:
        Circular_buffer<uint8_t, MAX_RECV_BUFF_LENGTH> rec_buff;

        unify_link_frame_head_t frame_head;
        std::array<uint8_t, MAX_FRAME_LENGTH> frame_data{};
        uint8_t last_seq_id = 0xFF;

        uint64_t com_error_count = 0;
        uint64_t decode_error_count = 0;
        uint64_t success_count = 0;

    public:
        Unify_link_base() { frame_data.fill(0); }

        void parse_data_task()
        {
            while (rec_buff.used() >= sizeof(frame_head))
            {
                if (!_find_frame_head())
                {
                    break;
                }

                // 读取数据段
                const uint16_t payload_len = frame_head.length();
                if (payload_len > MAX_FRAME_DATA_LENGTH)
                {
                    // 非法长度，跳过本字节重新找头
                    rec_buff.pop_data(1);
                    continue;
                }

                rec_buff.read_data(frame_data.data(), payload_len, sizeof(frame_head));

                // === 数据长度检查 ===
                if (rec_buff.used() < sizeof(frame_head) + payload_len)
                {
                    break; // 数据不足，等待更多数据
                }

                // === CRC 校验 ===
                uint16_t crc16_calc = crc16_calculation(reinterpret_cast<uint8_t *>(&frame_head),
                                                        offsetof(unify_link_frame_head_t, crc16));
                crc16_calc = crc16_calculation(frame_data.data(), payload_len, crc16_calc);

                if (crc16_calc != frame_head.crc16)
                {
                    rec_buff.pop_data(1); // 跳过本字节重新找头
                    continue;
                }

                // === 序号检查 ===
                uint8_t expected = last_seq_id + 1;
                if (frame_head.seq_id != expected)
                    com_error_count += (frame_head.seq_id - expected) & 0xFF;
                last_seq_id = frame_head.seq_id;

                // 消费整帧
                rec_buff.pop_data(sizeof(frame_head) + payload_len);

                // 业务处理
                if (!handle_data(frame_head.component_id, frame_head.data_id, frame_data.data(), payload_len))
                    decode_error_count++;
                else
                    success_count++; // 成功帧计数
            }
        }

        inline void rev_data_push(const uint8_t *data, uint32_t len)
        {
            if (len == 0 || len > rec_buff.remain())
            {
                return; // 数据不足
            }

            rec_buff.push_data(data, len);
        }

    public:
        registered_item_t registered_map[256][256]{};

        void register_handle_data(uint8_t component_id, uint8_t data_id, void *dst, handle_data_func_t func,
                                  uint16_t length)
        {
            // 线程/中断安全约束：该函数建议仅在初始化阶段调用。
            // 运行时若与 parse_data_task() 并发修改 map，会造成未定义行为。
            registered_map[component_id][data_id].callback = std::move(func);
            registered_map[component_id][data_id].dst = dst;
            registered_map[component_id][data_id].payload_length = length;
        }

        bool handle_data(uint8_t component_id, uint8_t data_id, const uint8_t *data, uint16_t len)
        {
            // 未注册
            if (registered_map[component_id][data_id].dst == nullptr)
                return false;

            // 请求帧 返回请求数据
            if (len == 0)
            {
                return build_send_data(component_id, data_id,
                                       reinterpret_cast<const uint8_t *>(registered_map[component_id][data_id].dst),
                                       registered_map[component_id][data_id].payload_length);
            }

            // 长度不匹配
            if (registered_map[component_id][data_id].payload_length != len)
                return false;

            // 处理数据 or 复制数据到目标地址
            const auto &callback = registered_map[component_id][data_id].callback;
            if (callback)
                return callback(data, len);
            else
                std::memcpy(registered_map[component_id][data_id].dst, data, len);
            return true;
        }

    protected:
        Circular_buffer<uint8_t, MAX_RECV_BUFF_LENGTH> send_buff;
        uint8_t send_tmp[MAX_FRAME_LENGTH] = {0};
        uint8_t seq_id = 0;

    public:
        // Convenience helpers for components: send a trivially-copyable object/array as payload.
        // Usage example from components:
        //   link_base.send_packet<component_id>(DATA_ID, obj_or_array);
        template <uint8_t ComponentId, typename T>
        void send_packet(uint8_t data_id, const T &obj)
        {
            build_send_data(ComponentId, data_id, reinterpret_cast<const uint8_t *>(&obj), sizeof(obj));
        }

        template <uint8_t ComponentId, typename T, size_t N>
        void send_packet(uint8_t data_id, const T (&arr)[N])
        {
            build_send_data(ComponentId, data_id, reinterpret_cast<const uint8_t *>(arr), sizeof(arr));
        }

        uint16_t build_send_data(const uint8_t component_id, const uint8_t data_id, const uint8_t *data,
                                 const uint16_t len)
        {
            if (len > MAX_FRAME_DATA_LENGTH)
            {
                return 0; // 超出最大帧长
            }

            // 如果发送缓冲区空间不足，直接返回 0
            if (send_buff.remain() < sizeof(unify_link_frame_head_t) + len)
            {
                return 0;
            }

            unify_link_frame_head_t *head = reinterpret_cast<unify_link_frame_head_t *>(send_tmp);
            head->frame_header = FRAME_HEADER;
            head->component_id = component_id;
            head->data_id = data_id;
            // 默认 flags=0
            head->set_flags_and_length(0, len);
            head->seq_id = seq_id;

            this->seq_id = seq_id + 1;

            std::memcpy(send_tmp + sizeof(unify_link_frame_head_t), data, len);

            // 计算 CRC
            head->crc16 = crc16_calculation(send_tmp, offsetof(unify_link_frame_head_t, crc16));
            head->crc16 = crc16_calculation(send_tmp + sizeof(unify_link_frame_head_t), len, head->crc16);

            // 将数据塞入发送缓冲区
            send_buff.push_data(send_tmp, sizeof(unify_link_frame_head_t) + len);

            return sizeof(unify_link_frame_head_t) + len;
        }

        uint32_t send_buff_used() const { return send_buff.used(); }
        uint32_t send_buff_remain() const { return send_buff.remain(); }

        void send_buff_pop(uint8_t *data, uint32_t *len)
        {
            uint32_t available = send_buff.used();
            if (available < sizeof(unify_link_frame_head_t))
            {
                *len = 0;
                return; // 数据不足
            }

            send_buff.read_data(data, available);
            *len = available;
            send_buff.pop_data(available);
        }
    };
}; // namespace unify_link
#endif // UNIFY_LINK_HPP
