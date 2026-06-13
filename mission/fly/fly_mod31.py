import time
import math
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def send_body_velocity(vx, vy, vz, yaw_rate):
    """发送机体坐标系速度 (X:前, Y:右, Z:下)"""
    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111, # 仅速度和偏航速率有效
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate
    )

def get_current_pos():
    """获取当前本地 NED 坐标"""
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
    if msg:
        return [msg.x, msg.y, msg.z]
    return None

def get_distance(p1, p2):
    """计算两点间的 3D 距离"""
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def set_offboard_mode():
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0
    )

def arm():
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )

# --- 主程序 ---
print("等待心跳...")
connection.wait_heartbeat()

# 预热
for _ in range(50):
    send_body_velocity(0, 0, 0, 0)
    time.sleep(0.1)

set_offboard_mode()
time.sleep(0.2)
arm()
print("已解锁并进入 Offboard")

# 1. 距离控制起飞 (向上起飞 5 米)
target_alt = 5.0
print(f"正在起飞，目标高度: {target_alt}m")
start_pos = get_current_pos()
if start_pos:
    while True:
        curr_pos = get_current_pos()
        # Z轴向上是负的，所以这里取绝对值计算高度差
        dist = abs(curr_pos[2] - start_pos[2])
        if dist >= target_alt:
            print("到达起飞高度")
            break
        send_body_velocity(0, 0, -1.0, 0) # 1m/s 上升
        time.sleep(0.1)

# 2. 距离控制水平飞行 (向前飞行 10 米)
target_dist = 10.0
speed = 2.0
print(f"正在向前飞行，目标距离: {target_dist}m, 速度: {speed}m/s")
# 重新获取开始飞行时的坐标
move_start_pos = get_current_pos()
if move_start_pos:
    while True:
        curr_pos = get_current_pos()
        dist = get_distance(move_start_pos, curr_pos)
        
        if dist >= target_dist:
            print(f"已完成 {target_dist}m 飞行")
            break
            
        send_body_velocity(speed, 0, 0, 0) # 前进
        time.sleep(0.1)

# 3. 悬停并降落
print("悬停 2 秒后降落...")
for _ in range(20):
    send_body_velocity(0, 0, 0, 0)
    time.sleep(0.1)

connection.mav.command_long_send(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
)
print("任务结束")