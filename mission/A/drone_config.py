# drone_config.py
from pymavlink import mavutil
import time

def get_connection():
    # SITL 使用 14540，如果两个程序同时跑，建议使用两个不同的端口，或确保一个关闭后再开另一个
    conn = mavutil.mavlink_connection('udp:127.0.0.1:14540')
    conn.wait_heartbeat()
    print(f"系统 ID: {conn.target_system} 已连接")
    return conn

def send_position(conn, x, y, z, yaw):
    conn.mav.set_position_target_local_ned_send(
        0, conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b100111111000, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0
    )