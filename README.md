# PX4 无人机仿真工作空间

基于 [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) + Gazebo Harmonic 的无人机仿真项目。

针对 **CUADC**，中国大学生飞行器设计创新大赛，多旋翼无人机侦察与救援项目设计。

根据2025年比赛规则复刻标准比赛场地，供仿真使用。

同时提供基础飞行脚本，包括解锁，起飞，绝对位置飞行例程脚本，相对位置飞行例程脚本，供使用者快速了解无人机offboard自主飞行控制与实机部署。

- **飞控**: PX4 v1.17+ (SITL)
- **仿真器**: Gazebo Harmonic 8.11 (gz-sim8)
- **平台**: Ubuntu 22.04
- **机型**: X500 四旋翼

<img width="2256" height="1503" alt="26d2f2792d44289153ecfe62fc4a3438" src="https://github.com/user-attachments/assets/d5e861ee-4c72-416d-9190-6d835ec9e62f" />

## 目录结构

```
px4_ws/
├── gazebo_models/          # Gazebo 世界场景 & 模型
│   ├── worlds/             # .sdf 世界文件
│   │   ├── indoor_20x20.sdf    # 20m×20m 无屋顶室内
│   │   ├── CUADC_UAV01.sdf     # 300m 室外赛道（基础）
│   │   └── CUADC_UAV02.sdf     # 300m 室外赛道（含障碍物）
│   └── models/             # 自定义模型
│       ├── x500_cam/            # X500 + 下视RGB摄像头
│       ├── x500_cam_base/       # X500 基础模型（含摄像头）
│       ├── tube_D*_H*.obj       # 空心圆筒网格
│       └── ...
├── launch/                  # 一键启动脚本
│   ├── launch_indoor.sh         # 启动: indoor_20x20
│   ├── launch_cuadc.sh          # 启动: CUADC_UAV01
│   ├── launch_cuadc2.sh         # 启动: CUADC_UAV02
│   └── launch_cuadc2_cam.sh     # 启动: CUADC_UAV02 + 下视摄像头
├── base/                   # PX4 基础控制脚本
├── fly_mod/                # 飞行模式示例
├── plane/                  # 固定翼
├── mission/                # 任务飞行
├── hand/                   # 手势识别
├── cv/                     # 计算机视觉
└── README.md
```

## Gazebo 世界场景

### indoor_20x20 — 室内无顶房间

| 属性 | 值 |
|------|-----|
| 尺寸 | 20m × 20m |
| 墙壁 | 4面，高 3m，厚 0.2m |
| 屋顶 | 无（顶部开放） |
| 障碍物 | 2根柱子、低台、高箱、桌子 |
| 起降点 | 房间中心 (0, 0, 0.3) |

```bash
bash launch/launch_indoor.sh
```

### CUADC_UAV01 — 室外赛道（基础）

| 属性 | 值 |
|------|-----|
| 地面 | 300m × 300m 绿色平地 |
| 赛道 | 8m × 65m 深灰色跑道 (y: -3 ~ 62) |
| 标线 | 白色条纹 y=32, y=37, y=57 |
| 起降点 | 原点，橙色 Ø1m 圆形 + 白色 H |
| 无人机朝向 | +Y（逆时针偏航 90°） |

```bash
bash launch/launch_cuadc.sh
```

### CUADC_UAV02 — 室外赛道（含目标圆筒）

在 CUADC_UAV01 基础上增加：

**区域 1** (y: 32 ~ 37)：3 个白色水筒（下封上开，壁厚 3mm），高 30cm

| 编号 | 直径 | 壁厚 |
|------|------|------|
| C1 | 15cm | 3mm |
| C2 | 20cm | 3mm |
| C3 | 25cm | 3mm |

**区域 2** (y: 57 ~ 62)：5 个白色水筒（下封上开，壁厚 3mm），高 15cm，直径 20cm
- 比赛时其中 3 个筒内放置 12×12cm 红色危险化学品标识

```bash
bash launch/launch_cuadc2.sh
```

<img width="2256" height="1503" alt="d2d4e2c5a80fc4c644f5269ea6d8d378" src="https://github.com/user-attachments/assets/dd93814d-2687-49fe-a08f-939d905cb1da" />

### 单独预览场景（无无人机）

```bash
gz sim gazebo_models/worlds/CUADC_UAV02.sdf
```

### X500 下视摄像头 (x500_cam)

基于 X500 四旋翼，base_link 底部加装下视 RGB 摄像头：

| 参数 | 值 |
|------|-----|
| 分辨率 | 640 × 480 |
| 帧率 | 30 Hz |
| FOV | 60° (horizontal) |
| 安装位置 | base_link 底部，朝下 |
| Gazebo 话题 | `/world/CUADC_UAV02/model/x500_cam_0/link/base_link/sensor/downward_camera/image` |
| ROS2 话题 | `/downward_camera` |

模型位于 `gazebo_models/models/x500_cam/`，不修改 PX4-Autopilot 内任何文件。

```bash
bash launch/launch_cuadc2_cam.sh

# 在另一个终端查看摄像头画面
ros2 run rqt_image_view rqt_image_view /downward_camera
```

**快速模式**（Gazebo 不嵌摄像头面板，ROS2 桥接，适合 YOLO 调用）：

```bash
bash launch/launch_cuadc2_cam_fast.sh           # 正常 GUI
bash launch/launch_cuadc2_cam_fast.sh --headless # 无 GUI，最快
bash launch/launch_cuadc2_cam_fast.sh --cam      # 自动弹出 rqt 查看器
bash launch/launch_cuadc2_cam_fast.sh --headless --cam  # 无 GUI + rqt
```

摄像头画面通过 ROS2 `/downward_camera` 话题获取，可直接接入 YOLO。```

## 场景布局图

```
                      CUADC_UAV02
                        y=62
    ┌──────────────────────────┐
    │   ○ ○ ○  (5× 白色圆筒)    │  Region 2
    │   ○   ○  H=15cm D=20cm   │  y:57~62
    │  ██████████████████████  │  y=57
    │                          │
    │      ○ ○ ○               │  Region 1
    │      H=30cm              │  y:32~37
    │  ██████████████████████  │  y=32
    │                          │
    │         🟠H              │  y=0 起降
    └──────────────────────────┘
    x=-4                    x=4
    无人机朝 +Y ↑
```

<img width="864" height="879" alt="eab391e255bbcaff08f84d45166e6952" src="https://github.com/user-attachments/assets/c1b1ddfc-202c-4a65-928f-298b88720977" />

实际比赛时为平行双场地，两组同时进行，注意信号干扰。

## 环境依赖

```bash
# Gazebo Harmonic
sudo apt install gz-harmonic

# PX4 工具链
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

## 启动原理

启动脚本采用 **standalone 模式**，不修改 PX4 内部文件：

1. 脚本启动 `gz sim server` + `gz sim gui`，加载指定世界
2. 以 `PX4_GZ_STANDALONE=1` 启动 PX4，PX4 检测到已有 Gazebo 运行后自动接入
3. PX4 退出后自动关闭 Gazebo

原版 `make px4_sitl gz_x500`（在 PX4-Autopilot 目录下）不受影响，依然使用默认 `default.sdf` 世界。

## 传感器 & 环境配置

所有世界均已配置：
- 磁场矢量 (Zurich WMM)
- 球面坐标 (47.4°N, 8.5°E)
- 绝热大气模型
- IMU / 磁力计 / GPS / 气压计 / 空速传感器插件
