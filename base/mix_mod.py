import time
import math
import threading
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

# 全局控制变量
ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0.0, 0.0, 0.0, 0.0
stop_thread = False

# 定义模式常量
MODE_TIME = "TIME" # 基于时间控制
MODE_DIST = "DIST" # 基于距离控制

# --- 底层：30Hz 稳定指令发送线程 ---
def send_loop():
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    while not stop_thread:
        connection.mav.set_position_target_local_ned_send(
            0, connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111,
            0, 0, 0, 
            ctrl_vx, ctrl_vy, ctrl_vz, 
            0, 0, 0, 0, ctrl_yaw_rate
        )
        time.sleep(1/30.0)

# --- 辅助：状态获取 ---
def get_status():
    pos = connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    att = connection.recv_match(type='ATTITUDE', blocking=False)
    return pos, att

# --- 执行器 A：时间模式 ---
def run_time_step(vx, vy, vz, yr, duration):
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    print(f"[时间模式] Vz:{vz} Vy:{vy} Vx:{vx} | 持续:{duration}s")
    ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = vx, vy, vz, yr
    time.sleep(duration)
    ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0, 0, 0, 0

# --- 执行器 B：距离模式 ---
def run_dist_step(dx, dy, dz, dyaw, v, rv):
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    p, a = None, None
    while p is None or a is None: p, a = get_status()
    
    start_pos = (p.x, p.y, p.z)
    start_yaw = a.yaw
    target_yaw_rad = math.radians(dyaw)
    total_dist = math.sqrt(dx**2 + dy**2 + dz**2)
    
    print(f"[距离模式] 目标位移:{total_dist:.1f}m, 转向:{dyaw}deg")

    while True:
        curr_p, curr_a = get_status()
        if not curr_p or not curr_a: continue
        
        # 计算进度
        moved = math.sqrt((curr_p.x-start_pos[0])**2 + (curr_p.y-start_pos[1])**2 + (curr_p.z-start_pos[2])**2)
        yaw_err = target_yaw_rad - (curr_a.yaw - start_yaw)
        yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi

        # 判定结束
        if (moved >= total_dist or total_dist == 0) and (abs(yaw_err) < 0.02):
            ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0, 0, 0, 0
            break

        # 逻辑控制（先转后飞）
        if abs(yaw_err) > 0.02:
            ctrl_yaw_rate = max(min(yaw_err * 1.5, rv), -rv)
            ctrl_vx, ctrl_vz = 0, 0
        else:
            ctrl_yaw_rate = 0
            scale = min(1.0, (total_dist - moved) / 0.5) 
            ctrl_vx = (dx / total_dist) * v * scale if total_dist > 0 else 0
            ctrl_vz = (dz / total_dist) * v * scale if total_dist > 0 else 0
        time.sleep(0.02)

# --- 主程序 ---
if __name__ == "__main__":
    try:
        print("等待心跳...")
        connection.wait_heartbeat()
        t = threading.Thread(target=send_loop, daemon=True)
        t.start()

        # 1. 解锁与进入 Offboard
        time.sleep(1)
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(2)

        # 2. 混合任务列表
        # 模式 TIME: [MODE_TIME, vx, vy, vz, yaw_rate, duration]
        # 模式 DIST: [MODE_DIST, dx, dy, dz, dyaw, velocity, yaw_velocity]
        mixed_path = [
            [MODE_TIME, 0.0, 0, -1.0, 0.0, 5.0],    # 模式1：起飞（持续上升5秒）
            [MODE_DIST, 10.0, 0, 0.0, 0, 2.0, 0.0], # 模式2：精准向前飞 10米
            [MODE_DIST, 0.0, 0, 0.0, 90, 0, 0.5],   # 模式2：精准右转 90度
            [MODE_TIME, 1.0, 0, 0.0, 0.2, 5.0],    # 模式1：边走边转弯（持续5秒）
            [MODE_DIST, 5.0, 0, 0.0, 0, 1.5, 0.0],  # 模式2：精准向前飞 5米
        ]

        for p in mixed_path:
            mode = p[0]
            if mode == MODE_TIME:
                run_time_step(p[1], p[2], p[3], p[4], p[5])
            elif mode == MODE_DIST:
                run_dist_step(p[1], p[2], p[3], p[4], p[5], p[6])
            
            print("动作完成，稳定中...")
            time.sleep(1.0)

        # 3. 降落
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    except KeyboardInterrupt:
        print("中断退出")
    finally:
        stop_thread = True
