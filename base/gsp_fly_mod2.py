import time
import math
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def set_flight_speed(speed):
    """设置水平飞行速度 (m/s)"""
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0,
        1,          # 速度类型: 空速或地速 (1 为地速)
        speed,      # 目标速度 (m/s)
        -1,         # 节流阀 (不变)
        0, 0, 0, 0  # 备用参数
    )

def set_yaw_condition(yaw_deg, yaw_rate):
    """设置目标偏航角及转向速率 (deg/s)"""
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0,
        yaw_deg,    # 目标偏航角度 (0-360)
        yaw_rate,   # 偏航角速度 (度/秒)
        0,          # 方向: 0 为绝对角度, 1 为相对角度
        0,          # 相对/绝对 标志
        0, 0, 0     # 备用参数
    )

def send_gps_waypoint(lat, lon, alt, yaw=0):
    """发送全球坐标指令"""
    connection.mav.set_position_target_global_int_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000, 
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, yaw, 0
    )

def get_current_status():
    """获取当前 GPS 和 高度信息"""
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
    if msg:
        return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0
    return None, None, None

def get_distance_meters(lat1, lon1, lat2, lon2):
    """计算两点间的水平距离"""
    dlat = (lat2 - lat1) * 111320
    dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
    return math.sqrt(dlat**2 + dlon**2)

def takeoff_to_alt(target_alt):
    print(f"正在起飞...")
    lat, lon, _ = get_current_status()
    while lat is None:
        lat, lon, _ = get_current_status()
    
    # 预热
    for _ in range(50):
        send_gps_waypoint(lat, lon, target_alt)
        time.sleep(0.1)

    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.2)
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    
    while True:
        _, _, curr_alt = get_current_status()
        if curr_alt and curr_alt >= target_alt * 0.95:
            print(f"已到达起飞高度: {curr_alt:.2f}m")
            break
        send_gps_waypoint(lat, lon, target_alt)
        time.sleep(0.2)

# --- 主任务 ---
try:
    print("等待心跳...")
    connection.wait_heartbeat()

    # 1. 垂直起飞
    takeoff_to_alt(10.0)

    # 2. GPS 航点列表 [lat, lon, alt, yaw, speed, yaw_rate]
    # speed: 飞行速度 (m/s), yaw_rate: 转向速度 (deg/s)
    waypoints = [
        [47.3977422, 8.5455939, 10.0, 0,   2.0, 30],
        [47.3970366, 8.5460595, 10.0, 144, 1.5, 45],
        [47.3967160, 8.5471457, 10.0, 72,  3.0, 20],
        [47.3962803, 8.5462791, 10.0, 216, 2.0, 30],
        [47.3951941, 8.5459585, 10.0, 144, 2.0, 30],
        [47.3958997, 8.5454929, 10.0, 288, 2.0, 30],
        [47.3951941, 8.5450273, 10.0, 216, 2.0, 30],
        [47.3962803, 8.5447068, 10.0, 360, 2.0, 30],
        [47.3967160, 8.5438402, 10.0, 288, 2.0, 30],
        [47.3970366, 8.5449264, 10.0, 72,  2.0, 30],
        [47.3977422, 8.5455939, 10.0, 0,   1.5, 20],
    ]

    for wp in waypoints:
        t_lat, t_lon, t_alt, t_yaw, t_speed, t_yaw_rate = wp
        print(f"前往航点: Lat:{t_lat}, Lon:{t_lon} | 速度:{t_speed}m/s, 转向:{t_yaw_rate}°/s")

        # 应用当前航点的速度和转向参数
        set_flight_speed(t_speed)
        set_yaw_condition(t_yaw, t_yaw_rate)

        # 闭环距离判断
        while True:
            c_lat, c_lon, c_alt = get_current_status()
            if c_lat is None: continue

            dist = get_distance_meters(c_lat, c_lon, t_lat, t_lon)
            alt_err = abs(c_alt - t_alt)

            # 到达判定条件 (1.0米以内)
            if dist < 1.0 and alt_err < 0.5:
                print("到达航点。")
                break

            send_gps_waypoint(t_lat, t_lon, t_alt, t_yaw)
            time.sleep(0.2)

    print("航线任务结束，降落...")
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

except KeyboardInterrupt:
    print("用户中断")
