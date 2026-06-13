import time
from pymavlink import mavutil

# 1. 建立连接
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')
connection.wait_heartbeat()
print(f"检测到系统 {connection.target_system}，准备执行任务降落...")

def set_param(param_id, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        param_id.encode('utf-8'),
        float(param_value),
        param_type
    )
    time.sleep(0.1)

# --- 核心修复：强制屏蔽所有地形检查 ---
print("正在屏蔽地形失效保护...")
set_param('FW_LND_USE_TER', 0)    # 禁用降落地形依赖
set_param('FW_LND_ANG', 7.0)      # 增大下滑角，防止高度修正引起的震荡
set_param('NAV_DLL_CH_ALT', 0)    # 禁用任务间隙高度检查
set_param('COM_RC_IN_MODE', 1)    # 仿真环境下忽略遥控器丢失（防止干扰降落）

# 如果还是报地形错误，尝试这个底层开关（部分版本有效）
set_param('MPC_ALT_MODE', 0)      # 将高度模式设为基于气压计而非地形

# --- 针对“临门一脚”中断的深度参数修复 ---
print("正在执行深度参数注入，强制屏蔽触地阶段的地形校验...")

# 1. 核心：强制降落阶段使用气压计高度，不准找地形
set_param('FW_LND_USE_TER', 0)    

# 2. 关键：将地形超时的容忍度设为极大值，防止因为没数据而 Abort
set_param('LND_TER_TIMEOUT', 10.0) 

# 3. 关键：调整拉平（Flare）逻辑，让它更依赖气压高度
# 减小拉平高度（例如设为 2.5m），让它更贴近地面再抬头
set_param('FW_LND_FLALT', 2.5)   

# 4. 强制禁用“自动复飞”逻辑（针对地形丢失引发的复飞）
# 注意：部分版本中此参数名为 FW_LND_ABORT
set_param('FW_LND_EARLYCFG', 0)   

# 5. 增大着陆点的垂直误差容忍度
set_param('NAV_FW_ALTL_CH', 10.0) 

print("参数注入完成，现在即使没有测距仪，飞机也会根据气压计撞向地面（着陆）。")

def start_mission():
    """触发已绘制的任务"""
    # PX4 Mission Mode: Main 4, Sub 4
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4, 4, 0, 0, 0, 0
    )
    print("已切换至 MISSION 模式，开始执行绘制的降落航线。")

# 启动任务
start_mission()

# 监控降落过程
try:
    while True:
        # 实时捕获报错，看看除了地形还有什么在阻碍
        msg = connection.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            print(f"\n[PX4 警告]: {msg.text}")
            if "Land aborted" in msg.text:
                print("检测到降落被拒绝！尝试强制发送紧急降落指令...")
                # 如果任务被中断，最后一招：强制切换到 LAND 模式忽略任务逻辑
                connection.mav.command_long_send(
                    connection.target_system, connection.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
                )

        pos = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos:
            alt = pos.relative_alt / 1000.0
            print(f"执行任务中 - 相对高度: {alt:.2f}m", end='\r')
            
            # 检查是否落地加锁
            hb = connection.recv_match(type='HEARTBEAT', blocking=False)
            if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) and alt < 1.0:
                print("\n[成功] 飞机已根据绘制点着陆并加锁。")
                break
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n停止监控。")