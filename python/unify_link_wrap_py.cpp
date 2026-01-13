#include "component/encoder_link.hpp"
#include "component/motor_link.hpp"
#include "component/update_Link.hpp"
#include "unify_link.hpp"

#include <algorithm>
#include <cstring>
#include <iterator>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
using namespace unify_link;

namespace
{
    template <typename T, size_t N>
    std::vector<T> copy_array(const T (&src)[N])
    {
        return std::vector<T>(src, src + N);
    }

    template <typename T, size_t N>
    void assign_array(T (&dst)[N], const std::vector<T> &src, const char *name)
    {
        if (src.size() != N)
        {
            throw std::invalid_argument(std::string("Expected ") + std::to_string(N) + " items for " + name);
        }
        std::copy(src.begin(), src.end(), dst);
    }

    std::string char_array_to_string(const char *src, size_t max_len)
    {
        return std::string(src, strnlen(src, max_len));
    }

    void assign_char_array(char *dst, size_t max_len, const std::string &value)
    {
        std::memset(dst, 0, max_len);
        const size_t copy_len = std::min(value.size(), max_len - 1);
        std::memcpy(dst, value.data(), copy_len);
    }

    bool push_recv_data(Unify_link_base &base, const py::bytes &data)
    {
        std::string view = data;
        const uint32_t len = static_cast<uint32_t>(view.size());
        if (len == 0 || len > base.rec_buff.remain())
        {
            return false;
        }

        base.rev_data_push(reinterpret_cast<const uint8_t *>(view.data()), len);
        return true;
    }

    py::bytes pop_send_buffer(Unify_link_base &base)
    {
        const uint32_t available = base.send_buff_used();
        if (available == 0)
        {
            return py::bytes();
        }

        std::vector<uint8_t> tmp(available);
        uint32_t written = 0;
        base.send_buff_pop(tmp.data(), &written);
        return py::bytes(reinterpret_cast<const char *>(tmp.data()), static_cast<py::ssize_t>(written));
    }

    uint16_t build_send_data_bytes(Unify_link_base &base, uint8_t component_id, uint8_t data_id,
                                   const py::bytes &payload)
    {
        std::string view = payload;
        const auto len = static_cast<uint16_t>(view.size());
        return base.build_send_data(component_id, data_id, reinterpret_cast<const uint8_t *>(view.data()), len);
    }

} // namespace

PYBIND11_MODULE(unify_link, m)
{
    m.doc() = "Pybind11 bindings for Unify_Link";

    m.attr("COMPONENT_ID_SYSTEM") = py::int_(COMPONENT_ID_SYSTEM);
    m.attr("COMPONENT_ID_MOTORS") = py::int_(COMPONENT_ID_MOTORS);
    m.attr("COMPONENT_ID_UPDATE") = py::int_(COMPONENT_ID_UPDATE);
    m.attr("COMPONENT_ID_ENCODERS") = py::int_(COMPONENT_ID_ENCODERS);
    m.attr("COMPONENT_ID_EXAMPLES") = py::int_(COMPONENT_ID_EXAMPLES);
    m.attr("FRAME_HEADER") = py::int_(FRAME_HEADER);
    m.attr("MAX_FRAME_DATA_LENGTH") = py::int_(MAX_FRAME_DATA_LENGTH);
    m.attr("MAX_FRAME_LENGTH") = py::int_(MAX_FRAME_LENGTH);

    py::class_<unify_link_frame_head_t>(m, "FrameHead")
        .def(py::init<>())
        .def_readwrite("frame_header", &unify_link_frame_head_t::frame_header)
        .def_readwrite("seq_id", &unify_link_frame_head_t::seq_id)
        .def_readwrite("component_id", &unify_link_frame_head_t::component_id)
        .def_readwrite("data_id", &unify_link_frame_head_t::data_id)
        .def_property("length", &unify_link_frame_head_t::length,
                      [](unify_link_frame_head_t &self, uint16_t len) { self.set_length(len); })
        .def_property("flags", &unify_link_frame_head_t::flags,
                      [](unify_link_frame_head_t &self, uint8_t flags) { self.set_flags(flags); })
        .def_readwrite("payload_length_and_sign", &unify_link_frame_head_t::payload_length_and_sign)
        .def_readwrite("crc16", &unify_link_frame_head_t::crc16);

    py::class_<Unify_link_base>(m, "UnifyLinkBase")
        .def(py::init<>())
        .def("parse_data_task", &Unify_link_base::parse_data_task, "Parse received buffer and dispatch frames")
        .def("rev_data_push", &push_recv_data, py::arg("data"),
             "Push raw bytes into the receive buffer. Returns False if the data does not fit.")
        .def("build_send_data", &build_send_data_bytes, py::arg("component_id"), py::arg("data_id"), py::arg("payload"),
             "Build a packet into the send buffer from raw payload bytes")
        .def("pop_send_buffer", &pop_send_buffer,
             "Pop all buffered outbound bytes as a Python bytes object (empties the buffer)")
        .def_property_readonly("send_buff_used", &Unify_link_base::send_buff_used)
        .def_property_readonly("send_buff_remain", &Unify_link_base::send_buff_remain)
        .def_readonly("last_seq_id", &Unify_link_base::last_seq_id)
        .def_readonly("com_error_count", &Unify_link_base::com_error_count)
        .def_readonly("decode_error_count", &Unify_link_base::decode_error_count)
        .def_readonly("success_count", &Unify_link_base::success_count);

    // Encoder bindings
    py::enum_<Encoder_link_t::ErrorCode>(m, "EncoderErrorCode")
        .value("OK", Encoder_link_t::ErrorCode::OK)
        .value("OVERFLOW_ERR", Encoder_link_t::ErrorCode::OVERFLOW_ERR)
        .value("MAGNET_TOO_STRONG", Encoder_link_t::ErrorCode::MAGNET_TOO_STRONG)
        .value("MAGNET_TOO_WEAK", Encoder_link_t::ErrorCode::MAGNET_TOO_WEAK)
        .value("INTERNAL_ERR", Encoder_link_t::ErrorCode::INTERNAL_ERR);

    py::class_<Encoder_link_t::encoder_basic_t>(m, "EncoderBasic")
        .def(py::init<>())
        .def_readwrite("position", &Encoder_link_t::encoder_basic_t::position)
        .def_readwrite("velocity", &Encoder_link_t::encoder_basic_t::velocity)
        .def_readwrite("error_code", &Encoder_link_t::encoder_basic_t::error_code);

    py::class_<Encoder_link_t::encoder_info_t>(m, "EncoderInfo")
        .def(py::init<>())
        .def_readwrite("encoder_id", &Encoder_link_t::encoder_info_t::encoder_id)
        .def_readwrite("resolution", &Encoder_link_t::encoder_info_t::resolution)
        .def_readwrite("max_velocity", &Encoder_link_t::encoder_info_t::max_velocity)
        .def_readwrite("max_position", &Encoder_link_t::encoder_info_t::max_position)
        .def_readwrite("run_time", &Encoder_link_t::encoder_info_t::run_time)
        .def_property(
            "model", [](const Encoder_link_t::encoder_info_t &self)
            { return char_array_to_string(self.model, sizeof(self.model)); },
            [](Encoder_link_t::encoder_info_t &self, const std::string &value)
            { assign_char_array(self.model, sizeof(self.model), value); })
        .def_property(
            "serial", [](const Encoder_link_t::encoder_info_t &self)
            { return std::vector<uint8_t>(self.serial, self.serial + sizeof(self.serial)); },
            [](Encoder_link_t::encoder_info_t &self, const std::vector<uint8_t> &serial)
            { assign_array(self.serial, serial, "serial"); })
        .def_readwrite("firmware_version", &Encoder_link_t::encoder_info_t::firmware_version);

    py::class_<Encoder_link_t::encoder_setting_t>(m, "EncoderSetting")
        .def(py::init<>())
        .def_readwrite("feedback_interval", &Encoder_link_t::encoder_setting_t::feedback_interval)
        .def_readwrite("reset_id", &Encoder_link_t::encoder_setting_t::reset_id);

    py::class_<Encoder_link_t>(m, "EncoderLink")
        .def(py::init<Unify_link_base &>(), py::arg("link_base"), py::keep_alive<1, 2>())
        .def_property(
            "encoder_basic", [](Encoder_link_t &self) { return copy_array(self.encoder_basic); },
            [](Encoder_link_t &self, const std::vector<Encoder_link_t::encoder_basic_t> &values)
            { assign_array(self.encoder_basic, values, "encoder_basic"); })
        .def_property(
            "encoder_info", [](Encoder_link_t &self) { return self.encoder_info; },
            [](Encoder_link_t &self, const Encoder_link_t::encoder_info_t &value) { self.encoder_info = value; })
        .def_property(
            "encoder_setting", [](Encoder_link_t &self) { return self.encoder_setting; },
            [](Encoder_link_t &self, const Encoder_link_t::encoder_setting_t &value) { self.encoder_setting = value; })
        .def_readonly_static("component_id", &Encoder_link_t::component_id)
        .def_readonly_static("MAX_ENCODERS", &Encoder_link_t::MAX_ENCODERS);

    // Motor_link_t bindings
    py::enum_<Motor_link_t::ErrorCode>(m, "MotorErrorCode")
        .value("OK", Motor_link_t::ErrorCode::OK)
        .value("OVER_HEAT_ERR", Motor_link_t::ErrorCode::OVER_HEAT_ERR)
        .value("INTERNAL_ERR", Motor_link_t::ErrorCode::INTERNAL_ERR);

    py::enum_<Motor_link_t::MotorMode>(m, "MotorMode")
        .value("CURRENT_CONTROL", Motor_link_t::MotorMode::CURRENT_CONTROL)
        .value("SPEED_CONTROL", Motor_link_t::MotorMode::SPEED_CONTROL)
        .value("POSITION_CONTROL", Motor_link_t::MotorMode::POSITION_CONTROL)
        .value("MIT_CONTROL", Motor_link_t::MotorMode::MIT_CONTROL);

    py::class_<Motor_link_t::motor_basic_t>(m, "MotorBasic")
        .def(py::init<>())
        .def_readwrite("position", &Motor_link_t::motor_basic_t::position)
        .def_readwrite("speed", &Motor_link_t::motor_basic_t::speed)
        .def_readwrite("current", &Motor_link_t::motor_basic_t::current)
        .def_readwrite("temperature", &Motor_link_t::motor_basic_t::temperature)
        .def_readwrite("error_code", &Motor_link_t::motor_basic_t::error_code);

    py::class_<Motor_link_t::motor_info_t>(m, "MotorInfo")
        .def(py::init<>())
        .def_readwrite("motor_id", &Motor_link_t::motor_info_t::motor_id)
        .def_readwrite("ratio", &Motor_link_t::motor_info_t::ratio)
        .def_readwrite("max_speed", &Motor_link_t::motor_info_t::max_speed)
        .def_readwrite("max_current", &Motor_link_t::motor_info_t::max_current)
        .def_readwrite("torque_constant", &Motor_link_t::motor_info_t::torque_constant)
        .def_readwrite("max_position", &Motor_link_t::motor_info_t::max_position)
        .def_readwrite("run_time", &Motor_link_t::motor_info_t::run_time)
        .def_property(
            "model",
            [](const Motor_link_t::motor_info_t &self) { return char_array_to_string(self.model, sizeof(self.model)); },
            [](Motor_link_t::motor_info_t &self, const std::string &value)
            { assign_char_array(self.model, sizeof(self.model), value); })
        .def_property(
            "serial", [](const Motor_link_t::motor_info_t &self)
            { return std::vector<uint8_t>(self.serial, self.serial + sizeof(self.serial)); },
            [](Motor_link_t::motor_info_t &self, const std::vector<uint8_t> &serial)
            { assign_array(self.serial, serial, "serial"); })
        .def_readwrite("firmware_version", &Motor_link_t::motor_info_t::firmware_version);

    py::class_<Motor_link_t::motor_settings_t>(m, "MotorSettings")
        .def(py::init<>())
        .def_readwrite("feedback_interval", &Motor_link_t::motor_settings_t::feedback_interval)
        .def_readwrite("reset_id", &Motor_link_t::motor_settings_t::reset_id)
        .def_property(
            "mode", [](const Motor_link_t::motor_settings_t &self) { return self.mode; },
            [](Motor_link_t::motor_settings_t &self, Motor_link_t::MotorMode mode) { self.mode = mode; });

    py::class_<Motor_link_t::motor_set_t>(m, "MotorSet")
        .def(py::init<>())
        .def_readwrite("motor_set", &Motor_link_t::motor_set_t::motor_set)
        .def_readwrite("motor_set_extra", &Motor_link_t::motor_set_t::motor_set_extra)
        .def_readwrite("motor_set_extra2", &Motor_link_t::motor_set_t::motor_set_extra2);

    py::class_<Motor_link_t>(m, "MotorLink")
        .def(py::init<Unify_link_base &>(), py::arg("link_base"), py::keep_alive<1, 2>())
        .def_property(
            "motor_basic", [](Motor_link_t &self) { return copy_array(self.motor_basic); },
            [](Motor_link_t &self, const std::vector<Motor_link_t::motor_basic_t> &values)
            { assign_array(self.motor_basic, values, "motor_basic"); })
        .def_property(
            "motor_info", [](Motor_link_t &self) { return copy_array(self.motor_info); },
            [](Motor_link_t &self, const std::vector<Motor_link_t::motor_info_t> &values)
            { assign_array(self.motor_info, values, "motor_info"); })
        .def_property(
            "motor_settings", [](Motor_link_t &self) { return copy_array(self.motor_settings); },
            [](Motor_link_t &self, const std::vector<Motor_link_t::motor_settings_t> &values)
            { assign_array(self.motor_settings, values, "motor_settings"); })
        .def_property(
            "motor_set", [](Motor_link_t &self) { return copy_array(self.motor_set); },
            [](Motor_link_t &self, const std::vector<Motor_link_t::motor_set_t> &values)
            { assign_array(self.motor_set, values, "motor_set"); })
        .def_readwrite("on_motor_info_updated", &Motor_link_t::on_motor_info_updated)
        .def_readwrite("on_motor_settings_updated", &Motor_link_t::on_motor_settings_updated)
        .def_readonly_static("component_id", &Motor_link_t::component_id)
        .def_readonly_static("MAX_MOTORS", &Motor_link_t::MAX_MOTORS);

    // Update bindings
    py::class_<Update_Link_t::firmware_info_t>(m, "FirmwareInfo")
        .def(py::init<>())
        .def_property(
            "firmware_data", [](const Update_Link_t::firmware_info_t &self)
            { return std::vector<uint8_t>(self.firmware_data, self.firmware_data + sizeof(self.firmware_data)); },
            [](Update_Link_t::firmware_info_t &self, const std::vector<uint8_t> &data)
            { assign_array(self.firmware_data, data, "firmware_data"); });

    py::class_<Update_Link_t::firmware_crc_t>(m, "FirmwareCRC")
        .def(py::init<>())
        .def_readwrite("crc16", &Update_Link_t::firmware_crc_t::crc16);

    py::class_<Update_Link_t>(m, "UpdateLink")
        .def(py::init<Unify_link_base &>(), py::arg("link_base"), py::keep_alive<1, 2>())
        .def_property(
            "firmware_info", [](Update_Link_t &self) { return self.firmware_info; },
            [](Update_Link_t &self, const Update_Link_t::firmware_info_t &value) { self.firmware_info = value; })
        .def_property(
            "firmware_crc", [](Update_Link_t &self) { return self.firmware_crc; },
            [](Update_Link_t &self, const Update_Link_t::firmware_crc_t &value) { self.firmware_crc = value; })
        .def("send_firmware_info", py::overload_cast<>(&Update_Link_t::send_firmware_info))
        .def("send_firmware_crc", py::overload_cast<>(&Update_Link_t::send_firmware_crc))
        .def_readonly_static("component_id", &Update_Link_t::component_id);
}
