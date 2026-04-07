import time
from pymavlink import mavutil
#机体位置控制，相对于机身方向飞行，速度控制模式
# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def send_body_velocity(vx, vy, vz, yaw_rate):
    """
    发送机体坐标系下的速度指令 (Body Frame NED)
    X: 向前, Y: 向右, Z: 向下
    """
    connection.mav.set_position_target_local_ned_send(
        0,                                  # time_boot_ms
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED, # 核心：使用机体坐标系
        0b0000011111000111,                 # 掩码: 仅速度和偏航速率有效
        0, 0, 0,                            # 位置 (忽略)
        vx, vy, vz,                         # 速度 (m/s)
        0, 0, 0,                            # 加速度 (忽略)
        0, yaw_rate                         # 偏航 (忽略), 偏航速率 (rad/s)
    )

def set_offboard_mode():
    """切换到 Offboard 模式"""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, 6, 0, 0, 0, 0, 0
    )

def arm():
    """解锁"""
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

print("等待心跳...")
connection.wait_heartbeat()
print("已连接到 PX4!")

# --- 步骤 1: 预热 (非常重要) ---
# 在切换模式前，必须持续发送指令，否则 PX4 会拒绝切换
print("开始发送预热指令 (机体静止)...")
for _ in range(100):
    send_body_velocity(0, 0, 0, 0)
    time.sleep(0.1)

# --- 步骤 2: 切换模式 ---
print("尝试切换到 OFFBOARD 模式...")
set_offboard_mode()

# --- 步骤 3: 解锁 ---
# 稍微等待模式切换生效
time.sleep(0.5) 
print("正在解锁...")
arm()
time.sleep(1) # 等待电机转起来

# --- 步骤 4: 控制循环 ---

# 4.1 自动起飞 (机体坐标系 Z 轴向上是负数)
print("起飞中 (上升 5 秒)...")
start_time = time.time()
while time.time() - start_time < 5:
    send_body_velocity(0, 0, -1.0, 0) # 1m/s 上升
    time.sleep(0.1)

# 4.2 向前飞行
print("向前飞行 (2m/s, 持续 5 秒)...")
start_time = time.time()
while time.time() - start_time < 5:
    send_body_velocity(2.0, 0, 0, 0) # 2m/s 前进
    time.sleep(0.1)

# 4.3 悬停一下
print("悬停 2 秒...")
for _ in range(20):
    send_body_velocity(0, 0, 0, 0)
    time.sleep(0.1)

# --- 步骤 5: 降落 ---
print("正在切换到 LAND 模式...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)

print("任务完成。")
