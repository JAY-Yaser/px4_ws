from pymavlink import mavutil
import time

# 1. 建立连接 (根据实际连接方式修改：如 '/dev/ttyUSB0', 'udp:127.0.0.1:14550' 等)
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

# 等待心跳包以确认连接成功
print("等待飞控心跳...")
connection.wait_heartbeat()
print(f"检测到系统 (ID: {connection.target_system} 组件: {connection.target_component})")

def arm_drone():
    # 2. 发送解锁命令 (MAV_CMD_COMPONENT_ARM_DISARM)
    # 参数: 1 表示解锁, 0 表示加锁
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, # 确认标志
        1, # 参数 1: 1 为 Arm (解锁)
        0, 0, 0, 0, 0, 0 # 其他参数置 0
    )

    # 3. 等待并确认解锁状态
    print("正在发送解锁指令...")
    while True:
        # 接收 MAVLINK_MSG_ID_HEARTBEAT 消息
        msg = connection.recv_match(type='HEARTBEAT', blocking=True)
        
        # 检查系统状态是否已变为已解锁
        # base_mode 的第 128 位 (MAV_MODE_FLAG_SAFETY_ARMED) 代表锁定状态
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("--- 无人机已解锁成功！ ---")
            break
        else:
            print("等待解锁...")
            # 某些情况下可能需要重复发送解锁指令
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
        time.sleep(1)

if __name__ == "__main__":
    arm_drone()