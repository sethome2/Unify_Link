import struct

import unify_link as ul


def loopback(base: ul.UnifyLinkBase) -> None:
    tx = base.pop_send_buffer()
    if tx:
        base.rev_data_push(tx)
        base.parse_data_task()


def build_encoder_basic_payload() -> bytes:
    # encoder_basic_t: uint16 position, int32 velocity, uint8 error_code (packed)
    fmt = "<HiB"
    entry = struct.pack(fmt, 123, -456, int(ul.EncoderErrorCode.OK))
    padding = struct.pack(fmt, 0, 0, int(ul.EncoderErrorCode.OK))
    return entry + padding * (ul.EncoderLink.MAX_ENCODERS - 1)


def build_encoder_info_payload() -> bytes:
    # encoder_info_t: uint8 id, uint8 resolution, uint32 max_velocity, uint32 max_position,
    # uint32 run_time, char model[32], uint8 serial[12], uint32 firmware_version (packed)
    model = b"ENCODER_XYZ"
    model = model + b"\x00" * (32 - len(model))
    serial = bytes(range(12))
    return struct.pack("<BBIII32s12sI", 0, 14, 5000, 9000, 123, model, serial, 0x01020304)


def build_encoder_setting_payload() -> bytes:
    # encoder_setting_t: uint8 feedback_interval, uint8 reset_id (packed)
    return struct.pack("<BB", 10, 0)


def build_motor_pid_payload() -> bytes:
    # pid_t: uint8 motor_id + 3 * PIDParams_t (7 floats each), packed
    floats = [
        1.0, 0.1, 0.01, 0.0, 0.5, 1.5, 2.0,  # current_pid
        1.1, 0.2, 0.02, 0.0, 0.6, 1.6, 2.1,  # speed_pid
        1.2, 0.3, 0.03, 0.0, 0.7, 1.7, 2.2,  # position_pid
    ]
    return struct.pack("<B" + "f" * 21, 0, *floats)


def main() -> None:
    base = ul.UnifyLinkBase()
    motors = ul.MotorLink(base)
    enc = ul.EncoderLink(base)
    update = ul.UpdateLink(base)

    # Frame head smoke test
    head = ul.FrameHead()
    head.frame_header = ul.FRAME_HEADER
    head.seq_id = 1
    head.component_id = ul.COMPONENT_ID_MOTORS
    head.data_id = 1
    head.length = 8
    head.flags = 3
    _ = head.payload_length_and_sign

    # Encoder payloads via raw build_send_data (no Python send_* bindings)
    base.build_send_data(ul.COMPONENT_ID_ENCODERS, 1, build_encoder_basic_payload())
    base.build_send_data(ul.COMPONENT_ID_ENCODERS, 2, build_encoder_info_payload())
    base.build_send_data(ul.COMPONENT_ID_ENCODERS, 3, build_encoder_setting_payload())
    loopback(base)

    # Motor arrays (need full-length lists)
    motor_basic = [ul.MotorBasic() for _ in range(ul.MotorLink.MAX_MOTORS)]
    motor_basic[0].position = 100
    motor_basic[0].speed = -20
    motor_basic[0].current = 33
    motor_basic[0].temperature = 45
    motor_basic[0].error_code = ul.MotorErrorCode.OK
    motors.motor_basic = motor_basic

    motor_info = [ul.MotorInfo() for _ in range(ul.MotorLink.MAX_MOTORS)]
    motor_info[0].motor_id = 0
    motor_info[0].ratio = 6.5
    motor_info[0].max_speed = 120.0
    motor_info[0].max_current = 5.0
    motor_info[0].torque_constant = 0.11
    motor_info[0].max_position = 9000
    motor_info[0].run_time = 12
    motor_info[0].model = "MOTOR_XYZ"
    motor_info[0].serial = list(range(12))
    motor_info[0].firmware_version = 0x00010002
    motors.motor_info = motor_info

    motor_settings = [ul.MotorSettings() for _ in range(ul.MotorLink.MAX_MOTORS)]
    motor_settings[0].motor_id = 0
    motor_settings[0].feedback_interval = 5
    motor_settings[0].reset_id = 0
    motor_settings[0].mode = ul.MotorMode.CURRENT_CONTROL
    motors.motor_settings = motor_settings

    motor_set = [ul.MotorSet() for _ in range(ul.MotorLink.MAX_MOTORS)]
    motor_set[0].set = 1234
    motor_set[0].set_extra = 0
    motor_set[0].set_extra2 = 0
    motors.motor_set = motor_set

    pid = ul.MotorPID()
    pid.motor_id = 0
    pid.current_pid = ul.PIDParams()
    pid.speed_pid = ul.PIDParams()
    pid.position_pid = ul.PIDParams()
    motors.motor_pid = pid

    # Callbacks
    callbacks: dict[str, int] = {
        "motor_basic": 0,
        "motor_info": 0,
        "motor_settings": 0,
        "motor_set": 0,
        "motor_pid": 0,
    }

    def on_basic(_data: list[ul.MotorBasic]) -> None:
        callbacks["motor_basic"] += 1

    def on_info(_data: ul.MotorInfo) -> None:
        callbacks["motor_info"] += 1

    def on_settings(_data: ul.MotorSettings) -> None:
        callbacks["motor_settings"] += 1

    def on_set(_data: list[ul.MotorSet]) -> None:
        callbacks["motor_set"] += 1

    def on_pid(_data: ul.MotorPID) -> None:
        callbacks["motor_pid"] += 1

    motors.on_motor_basic_updated = on_basic
    motors.on_motor_info_updated = on_info
    motors.on_motor_settings_updated = on_settings
    motors.on_motor_set_updated = on_set
    motors.on_motor_pid_updated = on_pid

    # Send motors payloads
    motors.send_motor_basic_data()
    motors.send_motor_info_data(motor_info[0])
    motors.send_motor_setting_data(motor_settings[0])
    motors.send_motor_set_data()
    loopback(base)
    
    # Raw PID update (data_id = 5)
    base.build_send_data(ul.COMPONENT_ID_MOTORS, 5, build_motor_pid_payload())
    loopback(base)

    # Motor mode helpers
    motors.set_motor_mode(0, ul.MotorMode.CURRENT_CONTROL)
    motors.set_motor_current(0, 100, 5)
    motors.send_motor_set_data()
    loopback(base)

    motors.set_motor_mode(0, ul.MotorMode.SPEED_CONTROL)
    motors.set_motor_speed(0, 200)
    motors.send_motor_set_data()
    loopback(base)

    motors.set_motor_mode(0, ul.MotorMode.POSITION_CONTROL)
    motors.set_motor_position(0, 300, 10)
    motors.send_motor_set_data()
    loopback(base)

    motors.set_motor_mode(0, ul.MotorMode.MIT_CONTROL)
    motors.set_motor_mit(0, 400, 20, 3)
    motors.send_motor_set_data()
    loopback(base)

    # UpdateLink payloads
    update.firmware_info = ul.FirmwareInfo()
    update.firmware_info.firmware_data = list(range(256))
    update.firmware_crc = ul.FirmwareCRC()
    update.firmware_crc.crc16 = 0xBEEF
    update.send_firmware_info()
    update.send_firmware_crc()
    loopback(base)

    # Base send/pop/recv APIs
    base.build_send_data(ul.COMPONENT_ID_SYSTEM, 1, b"\x01\x02\x03")
    loopback(base)

    print("Motor setpoint:", motors.motor_set[0].set)
    print("Callbacks:", callbacks)
    print("Success count:", base.success_count)


if __name__ == "__main__":
    main()
