from pymavlink import mavutil
import time

# 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')

def get_drone_info():
    print("正在获取无人机状态...")
    try:
        while True:
            # 1. 获取全球位置信息 (GPS)
            # lat/lon 单位是 1e7, alt 是毫米(mm)
            gps_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            
            # 2. 获取 HUD 信息 (包含罗盘朝向 Heading)
            hud_msg = connection.recv_match(type='VFR_HUD', blocking=True, timeout=1.0)
            
            if gps_msg and hud_msg:
                lat = gps_msg.lat / 1e7
                lon = gps_msg.lon / 1e7
                alt = gps_msg.relative_alt / 1000.0  # 相对起飞点高度(m)
                heading = hud_msg.heading             # 0-360度，0为正北
                
                print(f"--- 状态更新 ---")
                print(f"经度: {lon:.7f}, 纬度: {lat:.7f}")
                print(f"相对高度: {alt:.2f} m")
                print(f"机头朝向: {heading}°")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("停止获取。")

if __name__ == "__main__":
    connection.wait_heartbeat()
    print("已连接!")
    get_drone_info()