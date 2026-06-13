# step1_takeoff.py
import time
import math
from drone_config import get_connection, send_position
from pymavlink import mavutil

def main():
    master = get_connection()
    target_z = -10.0 # 升空10米

    print("预热中...")
    for _ in range(50):
        send_position(master, 0, 0, 0, 0)
        time.sleep(0.1)

    print("切换 Offboard 并解锁...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0
    )
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )

    print("正在起飞...")
    while True:
        send_position(master, 0, 0, target_z, 0)
        # 检查是否到达高度
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
        if msg and abs(msg.z - target_z) < 0.3:
            print("起飞完成，稳态悬停中。请准备运行飞行任务脚本...")
            print("按 Ctrl+C 退出并交给下一个脚本（注意：退出后需立即启动任务脚本）")
            try:
                while True: # 维持悬停指令流
                    send_position(master, 0, 0, target_z, 0)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("停止起飞脚本。")
                break

if __name__ == "__main__":
    main()