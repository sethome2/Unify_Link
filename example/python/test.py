import unify_link as ul

base = ul.UnifyLinkBase()
motors = ul.MotorLink(base)
enc = ul.EncoderLink(base)

# 构造并获取待发送帧
print(f"Motor Setpoint before: {motors.motor_set.motor_set[0]}")
motors.motor_set.motor_set = [100] + [0] * \
    7  # Set first element to 100, others to 0
print(f"Motor Setpoint after: {motors.motor_set.motor_set[0]}")
motors.send_motor_set_current_data()
tx = base.pop_send_buffer()

# 推送接收数据并解析
base.rev_data_push(tx)          # 模拟环回
base.parse_data_task()
print(f"Motor Setpoint from parsed: {motors.motor_set.motor_set[0]}")
print(f"Success count: {base.success_count}")
