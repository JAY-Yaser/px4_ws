import math
import time
import numpy as np
from pymavlink import mavutil
#加入位置检测和了偏航角度检测，分移动点和转向点

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

def get_current_location(connection):
    """从飞控获取当前的 NED 本地坐标"""
    # 阻塞式接收最近的一条位置消息
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
    if not msg:
        return None
    return msg.x, msg.y, msg.z

def is_at_target(current, target, threshold=0.3):
    """判断是否到达目标点"""
    if current is None: return False
    
    # 计算三维空间的欧几里得距离
    dist = math.sqrt(
        (target[0] - current[0])**2 + 
        (target[1] - current[1])**2 + 
        (target[2] - current[2])**2
    )
    return dist < threshold

def get_current_status(connection):
    # 接收位置
    pos_msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    # 接收姿态（包含偏航角）
    att_msg = connection.recv_match(type='ATTITUDE', blocking=True)
    return pos_msg, att_msg

def is_fully_reached(curr_pos, curr_att, target_pos, target_yaw, pos_tol=0.3, yaw_tol=0.1):
    # 位置距离
    d_pos = math.sqrt(sum((p - t)**2 for p, t in zip(curr_pos, target_pos)))
    # 角度差（注意处理弧度 0 与 2pi 的衔接）
    d_yaw = abs(curr_att.yaw - target_yaw)
    
    return d_pos < pos_tol and d_yaw < yaw_tol

def is_fully_aligned(curr_pos, curr_att_yaw, target_pos, target_yaw, pos_tol=0.3, yaw_tol=0.1):
    """同时检查位置和偏航角是否到位"""
    if curr_pos is None: return False
    
    # 1. 计算位置距离
    dist = math.sqrt(
        (target_pos[0] - curr_pos[0])**2 + 
        (target_pos[1] - curr_pos[1])**2 + 
        (target_pos[2] - curr_pos[2])**2
    )
    
    # 2. 计算偏航角误差 (处理 0 到 2pi 的循环差值)
    yaw_diff = abs(target_yaw - curr_att_yaw)
    if yaw_diff > math.pi:
        yaw_diff = 2 * math.pi - yaw_diff
        
    return dist < pos_tol and yaw_diff < yaw_tol

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
    [0, 0, -10.0, math.radians(0)],       # 原地升空
    [50, 0, -10.0, math.radians(0)],       # 向北飞 5 米
    [50, 0, -10.0, math.radians(90)],       #转向
    [50, 50, -10.0, math.radians(90)],  # 转向正东飞
    [50, 50, -10.0, math.radians(180)],  # 转向正东
    [0, 50, -10.0, math.radians(180)], # 转向正南飞
    [0, 50, -10.0, math.radians(0)],
    [0, 0, -10.0, math.radians(0)]        # 回到起点
]

try:
    for point in path:
        target_x, target_y, target_z, target_yaw = point
        print(f"目标点: x={target_x}, y={target_y}, z={target_z}, yaw={math.degrees(target_yaw)}°")

        while True:
            # 1. 持续发送指令
            send_position_with_yaw(target_x, target_y, target_z, target_yaw)
            
            # 2. 获取当前完整状态
            # 注意：你需要合并获取位置和姿态，或者分别获取
            pos_msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
            att_msg = connection.recv_match(type='ATTITUDE', blocking=True, timeout=0.1)
            
            if pos_msg and att_msg:
                curr_pos = (pos_msg.x, pos_msg.y, pos_msg.z)
                curr_yaw = att_msg.yaw
                
                # 3. 严格判断：位置和角度都要到
                if is_fully_aligned(curr_pos, curr_yaw, (target_x, target_y, target_z), target_yaw):
                    print(f"精准到达航点与朝向！")
                    # 建议在此处多停留 0.5s，让飞机稳一下
                    time.sleep(0.5) 
                    break
            
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