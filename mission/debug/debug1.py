import time
import math
import threading
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

# 全局控制变量
ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0.0, 0.0, 0.0, 0.0
stop_thread = False

def send_loop():
    """ 独立的控制线程，确保 30Hz 稳定输出 """
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    while not stop_thread:
        connection.mav.set_position_target_local_ned_send(
            0, connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111,
            0, 0, 0, ctrl_vx, ctrl_vy, ctrl_vz, 0, 0, 0, 0, ctrl_yaw_rate
        )
        time.sleep(1/30.0) # 30Hz

def get_status():
    # 使用 non-blocking 方式获取最新消息，避免卡顿
    pos = connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    att = connection.recv_match(type='ATTITUDE', blocking=False)
    return pos, att

def execute_body_action(target_dx, target_dy, target_dz, target_yaw_deg, v, rv):
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    
    # 初始状态获取
    p, a = None, None
    while p is None or a is None:
        p, a = get_status()
    
    start_pos = (p.x, p.y, p.z)
    start_yaw = a.yaw
    target_yaw_rad = math.radians(target_yaw_deg)
    total_dist = math.sqrt(target_dx**2 + target_dy**2 + target_dz**2)

    print(f"执行动作: 距离={total_dist:.1f}m, 转向={target_yaw_deg}deg")

    # 状态标志
    yaw_done = False
    
    while True:
        curr_p, curr_a = get_status()
        if not curr_p or not curr_a:
            time.sleep(0.01)
            continue
        
        # 1. 计算误差
        moved_dist = math.sqrt((curr_p.x-start_pos[0])**2 + (curr_p.y-start_pos[1])**2 + (curr_p.z-start_pos[2])**2)
        yaw_err = target_yaw_rad - (curr_a.yaw - start_yaw)
        yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi

        # 2. 检查是否完成
        if (moved_dist >= total_dist or total_dist == 0) and (abs(yaw_err) < 0.02):
            ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0, 0, 0, 0
            break

        # 3. 逻辑控制
        if abs(yaw_err) > 0.02 and not yaw_done:
            # 转向阶段：平滑减速
            p_gain = 1.5
            rate = yaw_err * p_gain
            ctrl_yaw_rate = max(min(rate, rv), -rv)
            ctrl_vx, ctrl_vz = 0, 0
        else:
            # 位移阶段
            yaw_done = True # 标记转向已基本完成，进入位移
            ctrl_yaw_rate = 0
            if moved_dist < total_dist:
                # 简单的平滑加速：如果刚开始移动，速度分步提升
                scale = min(1.0, (total_dist - moved_dist) / 0.5) # 末端减速
                ctrl_vx = (target_dx / total_dist) * v * scale
                ctrl_vz = (target_dz / total_dist) * v * scale
            else:
                ctrl_vx, ctrl_vz = 0, 0

        time.sleep(0.02)

# --- 主程序 ---
try:
    print("等待心跳...")
    connection.wait_heartbeat()

    # 启动控制线程
    t = threading.Thread(target=send_loop)
    t.start()

    # 进入 OFFBOARD 并解锁
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
    # 航线 [dx, dy, dz, dyaw, v, rv]
    path = [
        # --- 1. 起飞阶段 ---
        [0, 0, -10.0, 0, 2.0, 0],   # 起飞：垂直上升/下降到 5米高度

        # --- 2. 五角星循环 (5次) ---
        
        # 第 1 条边
        [0, 0, 0, 144, 0, 0.5],    # 转向：向右转 144度 (准备画第一笔)
        [50.0, 0, 0, 0, 8.0, 0],   # 前进：直线飞行 10米
        
        # 第 2 条边
        [0, 0, 0, 144, 0, 0.2],    # 转向：再转 144度
        [50.0, 0, 0, 0, 8.0, 0],   # 前进
        
        # 第 3 条边
        [0, 0, 0, 144, 0, 0.2],    # 转向
        [50.0, 0, 0, 0, 8.0, 0],   # 前进
        
        # 第 4 条边
        [0, 0, 0, 144, 0, 0.2],    # 转向
        [50.0, 0, 0, 0, 8.0, 0],   # 前进
        
        # 第 5 条边 (闭合)
        [0, 0, 0, 144, 0, 0.2],    # 转向
        [50.0, 0, 0, 0, 8.0, 0],   # 前进 (回到起点)

        # --- 3. 结束阶段 (可选) ---
        # 如果需要恢复初始机头朝向，可以取消下面的注释
        # [0, 0, 0, 72, 0, 0.5],   # 修正航向 (总共转了5*144=720度，减去360*2=720，其实朝向已经复原)
    ]

    for p in path:
        execute_body_action(*p)
        time.sleep(0.5)

    # 降落
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

finally:
    stop_thread = True
    print("程序退出")