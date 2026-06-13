import math
import time
import cv2
import numpy as np
import mediapipe as mp
from pymavlink import mavutil

# ================= 配置参数 =================
TAKEOFF_ALTITUDE = -5.0   # 起飞高度 (NED: -5 表示上方5米)
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
DEADZONE_WIDTH = 200      # 中间死区宽度
MOVE_STEP = 0.5           # 每次循环目标点移动的距离 (米)，控制灵敏度
MAX_DIST = 20.0           # 最大飞行半径 (防止飞太远)

# 连接飞控
connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')
print("等待心跳...")
connection.wait_heartbeat()
print(f"检测到飞控系统 ID: {connection.target_system}")

# ================= MediaPipe 初始化 =================
# 尝试使用新版 API，如果失败请确保下载了 hand_landmarker.task
try:
    base_options = mp.tasks.BaseOptions(model_asset_path='hand_landmarker.task')
    options = mp.tasks.vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.7
    )
    hand_landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)
    USE_NEW_API = True
except Exception as e:
    print(f"MediaPipe 初始化警告: {e}")
    hand_landmarker = None
    USE_NEW_API = False

# ================= MAVLink 核心函数 (参考你的代码) =================

def send_position_target(x, y, z):
    """
    发送位置目标指令
    使用 MAV_FRAME_BODY_OFFSET_NED (机身坐标系)
    """
    connection.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # 关键：机身坐标系
        0b110111111000,             # 掩码：只控制位置 (x, y, z)，忽略速度和加速度
        x, y, z,                    # 目标位置
        0, 0, 0,                    # 速度 (忽略)
        0, 0, 0,                    # 加速度 (忽略)
        0, 0                        # 偏航角 (忽略，保持当前朝向)
    )

def takeoff_procedure(target_z):
    """
    复刻参考代码的起飞逻辑：预热 -> 切换模式 -> 解锁 -> 发送目标
    """
    print("--- 开始起飞序列 ---")
    
    # 1. 预热 (发送当前位置，防止跳变)
    print("发送起飞点预热指令 (50次)...")
    for i in range(50):
        send_position_target(0, 0, target_z)
        time.sleep(0.1)

    # 2. 切换 Offboard 模式
    print("切换 Offboard 模式...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0
    )
    time.sleep(1) # 等待模式切换确认

    # 3. 解锁
    print("解锁电机...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    
    # 4. 稳定等待
    print(f"正在上升至 {abs(target_z)} 米...")
    # 持续发送目标点，直到飞控响应并稳定
    start_time = time.time()
    while time.time() - start_time < 5: # 简单等待5秒让飞机稳定
        send_position_target(0, 0, target_z)
        time.sleep(0.1)
    print("起飞完成，进入悬停待机。")

# ================= 主程序逻辑 =================

try:
    # 1. 执行起飞
    takeoff_procedure(TAKEOFF_ALTITUDE)

    # 2. 初始化摄像头
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    if not cap.isOpened():
        print("无法打开摄像头")
        exit()

    # 初始目标点设为原点 (相对于起飞点)
    target_x, target_y, target_z = 0.0, 0.0, TAKEOFF_ALTITUDE

    print("--- 进入手势控制循环 ---")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break

        frame = cv2.flip(frame, 1) # 镜像
        h, w, _ = frame.shape
        center_x = w // 2
        deadzone_left = center_x - (DEADZONE_WIDTH // 2)
        deadzone_right = center_x + (DEADZONE_WIDTH // 2)

        # 默认状态
        control_status = "悬停"
        
        # 3. 手部检测
        if USE_NEW_API and hand_landmarker:
            image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
            timestamp = int(time.time() * 1000)
            detection_result = hand_landmarker.detect_async(image, timestamp)

            if detection_result.hand_landmarks:
                landmarks = detection_result.hand_landmarks[0]
                # 食指指尖 (索引 8)
                index_finger_tip = landmarks[8]
                landmark_x = int(index_finger_tip.x * w)
                
                # 绘制关键点
                cv2.circle(frame, (landmark_x, int(index_finger_tip.y * h)), 15, (0, 255, 0), -1)

                # 4. 逻辑控制 (基于位置偏移)
                if landmark_x < deadzone_left:
                    # 左侧：目标点向左移动 (Y轴负方向，因为是机身坐标系)
                    target_y -= MOVE_STEP
                    control_status = "向左飞"
                    cv2.putText(frame, "MOVE LEFT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                elif landmark_x > deadzone_right:
                    # 右侧：目标点向右移动 (Y轴正方向)
                    target_y += MOVE_STEP
                    control_status = "向右飞"
                    cv2.putText(frame, "MOVE RIGHT", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                else:
                    # 死区：保持当前目标点不变
                    control_status = "悬停/保持"
                    cv2.putText(frame, "HOVER", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "未检测到手", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
             cv2.putText(frame, "MediaPipe 未就绪", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 限制最大飞行距离 (安全保护)
        dist = math.sqrt(target_x**2 + target_y**2)
        if dist > MAX_DIST:
            scale = MAX_DIST / dist
            target_x *= scale
            target_y *= scale

        # 5. 发送位置指令 (核心：持续发送最新的目标点)
        send_position_target(target_x, target_y, target_z)

        # 6. 绘制 UI
        cv2.line(frame, (center_x, 0), (center_x, h), (255, 255, 255), 2)
        cv2.rectangle(frame, (deadzone_left, 0), (deadzone_right, h), (0, 255, 255), 2)
        # 显示当前目标坐标
        cv2.putText(frame, f"Target Y: {target_y:.1f}", (10, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Status: {control_status}", (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow('Drone Position Control', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n程序中断")
finally:
    print("正在降落...")
    # 发送降落指令
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0
    )
    cap.release()
    cv2.destroyAllWindows()