import time
from pymavlink import mavutil

# 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def send_gps_waypoint(lat, lon, alt, yaw=0):
    """
    发送全球坐标航点
    lat, lon: 浮点数度数 (如 47.397742)
    alt: 相对高度 (m)
    yaw: 目标偏航角 (0-360, 0为正北)
    """
    connection.mav.set_position_target_global_int_send(
        0,                                   # time_boot_ms
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # 使用相对起飞点的高度
        0b0000111111111000,                  # 掩码: 只保留位置
        int(lat * 1e7),                      # 纬度 * 1e7
        int(lon * 1e7),                      # 经度 * 1e7
        alt,                                 # 高度 (m)
        0, 0, 0,                             # 速度 (忽略)
        0, 0, 0,                             # 加速度 (忽略)
        yaw, 0                               # 偏航, 偏航速率
    )

def setup_offboard():
    print("等待心跳...")
    connection.wait_heartbeat()
    
    # Offboard 必须持续发送目标点才能切换成功
    # 先获取当前位置作为初始点，防止切换时飞机乱飞
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    curr_lat = msg.lat / 1e7
    curr_lon = msg.lon / 1e7
    
    print("预热发送当前位置...")
    for _ in range(50):
        send_gps_waypoint(curr_lat, curr_lon, 5.0) # 预设高度 5m
        time.sleep(0.1)

    # 切换模式并解锁
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
    time.sleep(0.5)
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    print("Offboard 模式已激活!")

# --- 任务开始 ---
try:
    setup_offboard()

    # 定义 GPS 航点列表 [lat, lon, alt, yaw]
    # 请根据你 SITL 地图的实际位置修改这里的坐标！
    waypoints = [
        [47.3977422, 8.5455939, 10.0, 0],   # 航点 1
        [47.3979422, 8.5457939, 10.0, 90],  # 航点 2
        [47.3977422, 8.5455939, 5.0, 180],  # 返回
    ]

    for wp in waypoints:
        print(f"目标航点: {wp[0]}, {wp[1]} 高度: {wp[2]}")
        # 持续发送该航点指令 10 秒（或者你可以加入距离判断逻辑）
        start_time = time.time()
        while time.time() - start_time < 10:
            send_gps_waypoint(wp[0], wp[1], wp[2], wp[3])
            time.sleep(0.2)

    print("任务完成，准备降落")
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                   mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

except KeyboardInterrupt:
    print("中断")