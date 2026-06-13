import time
import math
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def send_body_cmd(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
    """发送机体坐标系指令"""
    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000011111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate
    )

def get_status():
    pos = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
    att = connection.recv_match(type='ATTITUDE', blocking=True, timeout=0.1)
    return pos, att

def execute_body_action(target_dx, target_dy, target_dz, target_yaw_deg, v, rv):
    """
    执行动作：
    v:  前进/上升线速度 (m/s)
    rv: 最大转向角速度 (rad/s)
    """
    start_pos_msg, start_att_msg = get_status()
    while not start_pos_msg or not start_att_msg:
        start_pos_msg, start_att_msg = get_status()
    
    start_pos = (start_pos_msg.x, start_pos_msg.y, start_pos_msg.z)
    start_yaw = start_att_msg.yaw
    
    total_dist = math.sqrt(target_dx**2 + target_dy**2 + target_dz**2)
    target_yaw_rad = math.radians(target_yaw_deg)
    
    print(f">> 目标: 距离={total_dist:.2f}m, 转向={target_yaw_deg}°, 速度={v}m/s, 转向速度={rv}rad/s")

    while True:
        curr_pos_msg, curr_att_msg = get_status()
        if not curr_pos_msg or not curr_att_msg:
            continue
        
        # 1. 位移计算
        curr_pos = (curr_pos_msg.x, curr_pos_msg.y, curr_pos_msg.z)
        moved_dist = math.sqrt(sum((c - s)**2 for c, s in zip(curr_pos, start_pos)))
        
        # 2. 偏航角增量计算 (处理当前偏航相对于进入动作时的改变量)
        relative_yaw = curr_att_msg.yaw - start_yaw
        # 弧度差值归一化到 [-pi, pi]
        yaw_error = target_yaw_rad - relative_yaw
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # 3. 判定条件
        dist_reached = moved_dist >= total_dist if total_dist > 0 else True
        yaw_reached = abs(yaw_error) < 0.02  # 精度提高到约 1 度

        if dist_reached and yaw_reached:
            send_body_cmd(0, 0, 0, 0)
            break

        out_vx = 0.0
        out_vz = 0.0
        out_yaw_rate = 0.0

        # --- 转向逻辑 (增加平滑减速) ---
        if not yaw_reached:
            # 比例控制：当误差小于 0.2 弧度时开始减速，最小不低于 0.1 rad/s
            p_gain = 2.0 
            out_yaw_rate = yaw_error * p_gain
            
            # 限制在用户设定的 rv 以内
            if out_yaw_rate > rv: out_yaw_rate = rv
            if out_yaw_rate < -rv: out_yaw_rate = -rv
            
            # 设定死区最小速度，防止无限震荡
            if abs(out_yaw_rate) < 0.05: out_yaw_rate = 0.05 if yaw_error > 0 else -0.05

        # --- 位移逻辑 ---
        elif not dist_reached:
            if total_dist > 0:
                out_vx = (target_dx / total_dist) * v
                out_vz = (target_dz / total_dist) * v

        send_body_cmd(vx=out_vx, vz=out_vz, yaw_rate=out_yaw_rate)
        time.sleep(0.05) # 提高控制频率到 20Hz 增加稳定性

# --- 航线配置 ---

# 格式: [dx, dy, dz, d_yaw, v, rv]
# d_yaw 是相对当前朝向的旋转增量
body_path = [
    [0, 0, -5.0, 0, 1.0, 0],     # 原地起飞 3m
    [10.0, 0, 0, 0, 1.5, 0],      # 前进 5m
    [0, 0, 0, 90, 0, 0.4],       # 原地右转 90度，限速 0.4rad/s
    [20.0, 0, 0, 0, 1.5, 0],      # 向前飞行 5m
    [0, 0, 0, -180, 0, 0.6],     # 原地左转 180度，限速 0.6rad/s
    [0, 0, 5.0, 0, 0.8, 0],      # 原地下滑 3m
]

# --- 启动逻辑 ---
def setup():
    print("等待心跳...")
    connection.wait_heartbeat()
    # 预发指令
    for _ in range(50):
        send_body_cmd(0, 0, 0, 0)
        time.sleep(0.05)
    
    # 模式切换
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)
    # 解锁
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    print("Mission Started!")

try:
    setup()
    for action in body_path:
        dx, dy, dz, dyaw, v, rv = action
        execute_body_action(dx, dy, dz, dyaw, v, rv)
        time.sleep(0.8) # 动作间停顿，增加物理稳定性

    print("Mission Complete. Landing...")
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

except KeyboardInterrupt:
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)