import time
import math
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def send_gps_waypoint(lat, lon, alt, yaw=0):
    """
    发送全球坐标指令 (MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)
    lat, lon: 角度 (如 47.397)
    alt: 相对起飞点高度 (m)
    yaw: 偏航角 (0为正北)
    """
    connection.mav.set_position_target_global_int_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000, # 掩码：只控制位置
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, yaw, 0
    )

def get_current_gps():
    """实时获取当前 GPS 位置和高度"""
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
    if msg:
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
    return None, None, None

def takeoff_to_alt(target_alt):
    """
    安全起飞：获取当前点经纬度，原地垂直上升到指定高度
    """
    print(f"正在准备起飞...")
    # 1. 锁定起飞时的经纬度
    lat, lon, alt = get_current_gps()
    while lat is None:
        lat, lon, alt = get_current_gps()
        time.sleep(0.1)
    
    print(f"锁定起飞坐标: Lat:{lat}, Lon:{lon}，目标高度:{target_alt}m")

    # 2. 预热发送（PX4 要求切换 Offboard 前必须有数据流）
    for _ in range(50):
        send_gps_waypoint(lat, lon, target_alt)
        time.sleep(0.1)

    # 3. 切换模式并解锁
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
    # 4. 闭环上升判断
    print("开始原地垂直起飞...")
    while True:
        _, _, curr_alt = get_current_gps()
        if curr_alt is not None:
            if curr_alt >= target_alt * 0.95: # 到达目标高度的 95% 认为到位
                print(f"已到达安全高度: {curr_alt:.2f}m")
                break
        
        # 持续发送当前经纬度和目标高度，确保原地直上
        send_gps_waypoint(lat, lon, target_alt)
        time.sleep(0.2)

# --- 主任务 ---
try:
    print("等待心跳...")
    connection.wait_heartbeat()

    # 步骤 1: 独立垂直起飞 (设定起飞高度为 10 米)
    takeoff_to_alt(10.0)

    # 步骤 2: 执行 GPS 航点列表
    # 此时飞机会从 10 米高度 平移 飞向第一个航点
    waypoints = [
        [47.3977422, 8.5455939, 10.0, 0],     # 航点 1: 顶点 (起始点)
        [47.3970366, 8.5460595, 10.0, 144],   # 航点 2: 内凹点
        [47.3967160, 8.5471457, 10.0, 72],    # 航点 3: 顶点
        [47.3962803, 8.5462791, 10.0, 216],   # 航点 4: 内凹点
        [47.3951941, 8.5459585, 10.0, 144],   # 航点 5: 顶点
        [47.3958997, 8.5454929, 10.0, 288],   # 航点 6: 内凹点
        [47.3951941, 8.5450273, 10.0, 216],   # 航点 7: 顶点
        [47.3962803, 8.5447068, 10.0, 360],   # 航点 8: 内凹点
        [47.3967160, 8.5438402, 10.0, 288],   # 航点 9: 顶点
        [47.3970366, 8.5449264, 10.0, 72],    # 航点 10: 内凹点
        [47.3977422, 8.5455939, 10.0, 0],     # 航点 11: 回到顶点 (闭合)
    ]

    for wp in waypoints:
        t_lat, t_lon, t_alt, t_yaw = wp
        print(f"前往航点: Lat:{t_lat}, Lon:{t_lon}, Alt:{t_alt}")
        
        # 简单的到达判断：持续发送 15 秒（实际建议用距离判断）
        start_t = time.time()
        while time.time() - start_t < 15:
            send_gps_waypoint(t_lat, t_lon, t_alt, t_yaw)
            time.sleep(0.2)

    print("航线任务结束，自动降落...")
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

except KeyboardInterrupt:
    print("用户中断")