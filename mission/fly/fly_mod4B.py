import time
import math
import threading
from pymavlink import mavutil

# 1. 建立连接 (PX4 SITL)
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

# 全局变量：用于线程间通信
ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0.0, 0.0, 0.0, 0.0
stop_thread = False

# --- 底层：控制指令线程 ---
def send_loop():
    """ 
    以 30Hz 的高频率持续发送 MAVLink 指令。
    即便主程序在计算或等待，飞控也不会因为丢包而卡顿。
    """
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    while not stop_thread:
        connection.mav.set_position_target_local_ned_send(
            0, connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111, # 仅速度和偏航速率
            0, 0, 0, 
            ctrl_vx, ctrl_vy, ctrl_vz, 
            0, 0, 0, 0, ctrl_yaw_rate
        )
        time.sleep(1/30.0)

# --- 中层：状态获取与动作执行 ---
def get_status():
    """获取最新位置和姿态"""
    pos = connection.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    att = connection.recv_match(type='ATTITUDE', blocking=False)
    return pos, att

def execute_body_action(dx, dy, dz, dyaw, v, rv):
    """
    执行单条指令：
    dx, dy, dz: 机体坐标系位移 (m)
    dyaw: 转向角度增量 (度)
    v: 飞行线速度 (m/s)
    rv: 转向角速度 (rad/s)
    """
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    
    # 获取起始点状态
    p, a = None, None
    while p is None or a is None:
        p, a = get_status()
    
    start_pos = (p.x, p.y, p.z)
    start_yaw = a.yaw
    target_yaw_rad = math.radians(dyaw)
    total_dist = math.sqrt(dx**2 + dy**2 + dz**2)

    print(f"正在执行: 位移={total_dist:.1f}m, 转向={dyaw}°, 速度={v}m/s")

    while True:
        curr_p, curr_a = get_status()
        if not curr_p or not curr_a:
            continue
        
        # 1. 计算当前相对于起始点的完成进度
        moved_dist = math.sqrt((curr_p.x-start_pos[0])**2 + (curr_p.y-start_pos[1])**2 + (curr_p.z-start_pos[2])**2)
        yaw_err = target_yaw_rad - (curr_a.yaw - start_yaw)
        # 弧度归一化
        yaw_err = (yaw_err + math.pi) % (2 * math.pi) - math.pi

        # 2. 停止条件判断
        dist_reached = (moved_dist >= total_dist or total_dist == 0)
        yaw_reached = (abs(yaw_err) < 0.02) # 约 1 度误差

        if dist_reached and yaw_reached:
            ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0, 0, 0, 0
            break

        # 3. 速度分配逻辑
        if not yaw_reached:
            # 优先执行转向 (P控制)
            p_gain = 1.5
            ctrl_yaw_rate = max(min(yaw_err * p_gain, rv), -rv)
            ctrl_vx, ctrl_vz = 0, 0
        elif not dist_reached:
            # 转向完成后执行直线位移
            ctrl_yaw_rate = 0
            # 距离终点 0.5m 时开始减速，让停机更平滑
            scale = min(1.0, (total_dist - moved_dist) / 0.5) 
            ctrl_vx = (dx / total_dist) * v * scale if total_dist > 0 else 0
            ctrl_vz = (dz / total_dist) * v * scale if total_dist > 0 else 0

        time.sleep(0.02) # 逻辑刷新频率

# --- 上层：主程序流程 ---
if __name__ == "__main__":
    try:
        print("等待心跳...")
        connection.wait_heartbeat()
        
        # 启动后台发包线程
        t = threading.Thread(target=send_loop, daemon=True)
        t.start()

        # 1. 预热
        print("预热中...")
        time.sleep(2)

        # 2. 切模式 & 解锁
        print("切换 OFFBOARD 并解锁...")
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(2)

        # 3. 航线任务列表 [dx, dy, dz, dyaw, v, rv]
        path = [
            [0.0,  0, -5.0,  0,   1.0, 0.0],  # 起飞 (上升 5米)
            [10.0, 0,  0.0,  0,   2.0, 0.0],  # 前进 10米 (速度 2.0)
            [0.0,  0,  0.0,  90,  0.0, 0.5],  # 原地右转 90度 (角速度 0.5)
            [10.0, 0,  0.0,  0,   2.0, 0.0],  # 再前进 10米
            [0.0,  0,  0.0,  -90, 0.0, 0.5], # 转向回初始朝向
            [0.0,  0,  5.0,  0,   1.0, 0.0],  # 下降回到地面
        ]

        for p in path:
            execute_body_action(*p)
            print("动作完成，稳定 0.5 秒...")
            time.sleep(0.5)

        # 4. 降落
        print("任务结束，执行降落模式...")
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    except KeyboardInterrupt:
        print("\n用户强行中断！")
    finally:
        stop_thread = True
        print("程序已安全退出")