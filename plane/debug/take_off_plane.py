import time
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

print("等待心跳包...")
connection.wait_heartbeat()
print(f"检测到系统 (ID: {connection.target_system} 组件: {connection.target_component})")

def set_px4_mode(mode_name):
    """
    专门针对 PX4 的模式切换函数
    使用 MAV_CMD_DO_SET_MODE (176) 更加可靠
    """
    # PX4 的模式定义
    PX4_CUSTOM_MAIN_MODE_AUTO = 4
    PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF = 2
    PX4_CUSTOM_SUB_MODE_AUTO_LOITER = 3

    if mode_name == 'TAKEOFF':
        main_mode = PX4_CUSTOM_MAIN_MODE_AUTO
        sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF
    elif mode_name == 'LOITER':
        main_mode = PX4_CUSTOM_MAIN_MODE_AUTO
        sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER
    else:
        print(f"尚未定义模式: {mode_name}")
        return

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        main_mode,
        sub_mode,
        0, 0, 0, 0
    )
    print(f"指令已发送: 切换至 {mode_name}")

# 2. 执行流程
# 切换起飞模式
set_px4_mode('TAKEOFF')

# 3. 解锁
print("正在解锁...")
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

# 等待解锁成功
connection.motors_armed_wait()
print("解锁成功！固定翼正在滑跑...")

# 4. 循环读取高度，直到达到 30 米切换盘旋
try:
    while True:
        # 获取位置信息
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if not msg:
            continue
            
        alt = msg.relative_alt / 1000.0  # 毫米转米
        print(f"当前相对高度: {alt:.2f}m", end='\r')
        
        if alt >= 30:
            print(f"\n已达到预定高度 {alt:.2f}m，切换至盘旋模式")
            set_px4_mode('LOITER')
            break
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n用户中断")

print("任务完成，保持盘旋中...")