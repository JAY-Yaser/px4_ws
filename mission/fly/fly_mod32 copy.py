import time
import threading
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

# 全局控制变量
ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0.0, 0.0, 0.0, 0.0
stop_thread = False

# --- 底层：稳定指令发送线程 (30Hz) ---
def send_loop():
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    while not stop_thread:
        connection.mav.set_position_target_local_ned_send(
            0, connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111, # 仅速度和偏航速率有效
            0, 0, 0, 
            ctrl_vx, ctrl_vy, ctrl_vz, 
            0, 0, 0, 0, ctrl_yaw_rate
        )
        time.sleep(1/30.0)

# --- 中层：动作执行器 (基于时间) ---
def execute_time_action(vx, vy, vz, yaw_rate, duration):
    """
    根据给定的速度和角速度，持续运行指定的时间
    """
    global ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate
    
    print(f"执行动作: V({vx}, {vy}, {vz}), YawRate({yaw_rate}), 持续 {duration}秒")
    
    # 设置全局变量，send_loop 线程会自动获取并发送
    ctrl_vx = vx
    ctrl_vy = vy
    ctrl_vz = vz
    ctrl_yaw_rate = yaw_rate
    
    # 持续指定时间
    time.sleep(duration)
    
    # 动作结束，重置速度（悬停）
    ctrl_vx, ctrl_vy, ctrl_vz, ctrl_yaw_rate = 0, 0, 0, 0
    time.sleep(0.2) # 短暂稳定时间

# --- 主程序流程 ---
if __name__ == "__main__":
    try:
        print("等待心跳...")
        connection.wait_heartbeat()
        
        # 启动后台发包线程
        t = threading.Thread(target=send_loop, daemon=True)
        t.start()

        # 1. 模式准备与预热
        print("准备中...")
        time.sleep(1)
        
        # 切换 OFFBOARD 并解锁
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(2)

        # 2. 基于时间的航线任务列表
        # 格式: [vx, vy, vz, yaw_rate, duration]
        # vz: -1.0 为上升, yaw_rate: 单位 rad/s
        time_path = [
            [0.0,  0, -1.5,  0.0,  5.0],  # 动作 1: 以 1.5m/s 速度上升 4秒
            [2.0,  0,  0.0,  0.0,  5.0],  # 动作 2: 以 2.0m/s 速度向前飞 5秒
            [0.0,  0,  0.0,  0.5,  3.14], # 动作 3: 以 0.5rad/s 速度右转约 3.14秒 (转180度)
            [1.0,  0,  0.0,  0.0,  3.0],  # 动作 4: 向前飞 3秒 (此时已掉头)
            [0.0,  0,  1.0,  0.0,  2.0],  # 动作 5: 以 1.0m/s 速度下降 2秒
        ]

        # 3. 循环执行列表任务
        for p in time_path:
            execute_time_action(*p)
            print("动作完成，稳定中...")
            time.sleep(0.5)

        # 4. 任务结束降落
        print("执行自动降落...")
        connection.mav.command_long_send(connection.target_system, connection.target_component,
                                       mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        stop_thread = True
        print("程序退出")