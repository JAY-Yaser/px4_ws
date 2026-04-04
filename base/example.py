import math
import time
from pymavlink import mavutil

# 1. 建立连接 (SITL 默认地址，实机请修改为串口如 '/dev/ttyUSB0')
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("等待心跳...")
connection.wait_heartbeat()
print(f"检测到飞控系统 ID: {connection.target_system}")

def set_offboard_mode():
    """切换到 Offboard 模式"""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        1,      # Base Mode: 定位为 1 (Custom Mode)
        6,      # Custom Mode: PX4 中 6 代表 OFFBOARD
        0, 0, 0, 0, 0
    )

def arm():
    """解锁"""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

def send_position_target(x, y, z):
    connection.mav.set_position_target_local_ned_send(
        0,                                   # time_boot_ms (设置为 0 即可)
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b110111111000, 
        x, y, z,        
        0, 0, 0,        
        0, 0, 0,        
        0, 0            
    )

def send_position_with_yaw(x, y, z, yaw):
    """
    增加偏航角控制
    yaw: 弧度 (0 是正北, pi/2 是正东)
    """
    connection.mav.set_position_target_local_ned_send(
        0,
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b100111111000, # 掩码修改：取消屏蔽偏航角 (第11位)
        x, y, z,
        0, 0, 0, 0, 0, 0,
        yaw, 0
    )

# --- 主程序逻辑 ---

# 步骤 A: 预热发送目标值 (必须在切换 Offboard 前开始)
print("开始发送位置预热指令...")
for i in range(50):
    # 先发送在地面的位置，高度为 0
    send_position_target(0, 0, 0)
    time.sleep(0.1)

# 步骤 B: 切换到 Offboard 模式
print("尝试切换到 Offboard 模式...")
set_offboard_mode()

# 步骤 C: 解锁
print("正在解锁...")
arm()

# 步骤 D: 航线飞行
print("开始执行航线任务...")
# 定义航点列表 [x, y, z, yaw] yaw为转向角度
# 注意：z=-3 是保持 3 米高度
path = [
    [0, 0, -3.0, math.radians(0)],       # 原地升空
    [10, 0, -5.0, math.radians(0)],       # 向北飞 5 米
    [10, 0, -5.0, math.radians(90)],       #转向
    [10, 5, -5.0, math.radians(90)],  # 转向正东飞
    [10, 5, -5.0, math.radians(180)],  # 转向正东
    [0, 5, -3.0, math.radians(180)], # 转向正南飞
    [0, 5, -3.0, math.radians(0)],
    [0, 0, -3.0, 0]        # 回到起点
]

try:
    for point in path:
        target_x, target_y, target_z, target_yaw = point
        print(f"前往目标点: x={target_x}, y={target_y}, z={target_z}, yaw={target_yaw}")
        
        # 每个点持续飞行 5 秒（简单处理，实际应判断距离）
        for _ in range(50): 
            send_position_with_yaw(target_x, target_y, target_z, target_yaw)
            time.sleep(0.1)

    # 任务结束，原地降落
    print("任务完成，准备着陆...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )


except KeyboardInterrupt:
    print("程序停止，尝试加锁...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )