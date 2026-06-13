import time
from pymavlink import mavutil

# 1. 建立连接 (SITL 默认地址，实机请修改为串口如 '/dev/ttyUSB0')
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

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

# 步骤 D: 循环发送起飞到 3 米的指令
print("起飞中：目标高度 3 米...")
try:
    while True:
        # 在 NED 坐标系下，z 轴向下为正，向上为负。3米高即 z = -3
        send_position_target(0, 0, -3.0)
        
        # 实际开发中，这里可以加入判断逻辑：
        # 如果当前高度已经接近 -3，则打印“已到达目标高度，悬停中”
        
        time.sleep(0.1) # 以 10Hz 的频率维持 Offboard 指令流

except KeyboardInterrupt:
    print("程序停止，尝试加锁...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )