# Autocar — Gazebo 仿真环境设计文档

**日期：** 2026-06-17
**版本：** v1.0
**状态：** Approved

---

## 1. 概述

本文档定义 Autocar 自主小车的 Gazebo Ignition 仿真环境架构。纯 PC 端仿真，所有 16 个 ROS2 节点在本地运行，不依赖树莓派硬件，为算法验证和系统集成测试提供完整数字孪生环境。

**目标：** 逐层验证控制层 → 感知层 → 规划层 → 应用层，最终全栈联调。

---

## 2. Gazebo 环境

### 2.1 差分驱动小车模型 (URDF/SDF)

| 部件 | Gazebo 插件 | 发布话题 | 对应真实硬件 |
|------|------------|----------|-------------|
| 底盘 | `DiffDrivePlugin` | `/odom` | 电机 + 编码器 |
| 2D LiDAR | `GpuRayPlugin` | `/scan` | LD06 / YDLIDAR X4 |
| IMU | `IMUSensorPlugin` | `/imu` | MPU9250 |
| GPS | `NavSatPlugin` | `/gps/fix` | BN-880Q |
| 深度相机 | `DepthCameraPlugin` | `/camera/depth`, `/camera/rgb` | OAK-D Lite / D435 |

### 2.2 仿真世界

两级场景，渐进验证：

| 场景 | 文件 | 用途 |
|------|------|------|
| 测试障碍 | `worlds/test_obstacle.sdf` | 控制层 + 避障验证，若干立方体障碍 |
| 城市街道 | `worlds/city_street.sdf` | 规划层 + 全栈联调，车道线/路口/行人 |

### 2.3 文件结构

```
src/autocar_simulation/
├── package.xml
├── CMakeLists.txt
├── models/
│   └── autocar_car/
│       ├── model.sdf               # 完整 SDF 模型
│       └── materials/              # 材质贴图
├── worlds/
│   ├── test_obstacle.sdf
│   └── city_street.sdf
├── launch/
│   ├── sim_bringup.launch.py       # 启动 Gazebo + 机器人模型
│   └── autocar_full.launch.py      # 启动全部节点 + Gazebo
├── src/
│   └── simulation_bridge.cpp       # 仿真桥接节点
└── config/
    └── sim_params.yaml             # 仿真参数（噪声、时间等）
```

---

## 3. 系统拓扑

```
                    ┌─────────────────────────────────────┐
                    │         Gazebo Ignition              │
                    │  ┌──────────┐  ┌──────────┐          │
                    │  │DiffDrive │  │Lidar/IMU │          │
                    │  │ Plugin   │  │ Plugins  │          │
                    │  └────┬─────┘  └────┬─────┘          │
                    └───────┼─────────────┼─────────────────┘
                            │             │
                  /odom     │     /scan   │  /imu  /gps/fix  /camera/...
                            ▼             ▼
                    ┌─────────────────────────────────────┐
                    │      simulation_bridge               │
                    │ 话题映射 + 噪声注入 + use_sim_time   │
                    └──┬──────┬──────┬──────┬─────────────┘
                       │      │      │      │
             /odom     │/scan │/imu  │/gps  │/camera/...
                       ▼      ▼      ▼      ▼
    ┌────────────┐  ┌──────────────────────┐  ┌────────────┐
    │  控制层     │  │    感知层             │  │  规划层     │
    │ PID 控制   │  │  EKF融合  SLAM       │  │  A*  DWA   │
    │ 安全监控    │  │  传感器驱动(仿真模式)  │  │  行为树     │
    └──────┬─────┘  └──────────────────────┘  └──────┬─────┘
           │                                         │
           └───────────── /cmd_vel ──────────────────┘
                              │
                    Gazebo DiffDrive Plugin
```

### 3.1 桥接层职责

- **话题映射：** Gazebo 原始话题（`/gazebo/default/odom`）→ 标准话题（`/odom`）
- **噪声注入：** 传感器数据加高斯噪声，模拟真实硬件不完美
- **`use_sim_time` 管理：** 所有节点使用仿真时钟，确保 replay 一致性
- **`sim:=true` 参数：** 驱动层切换为仿真模式（输出到 Gazebo 而非实际 GPIO）

---

## 4. 分层验证流程

### 阶段 1 — 控制层验证

```
启动:  Gazebo + DiffDrive + test_obstacle.sdf
运行:  velocity_controller + steering_controller + safety_watchdog
输入:  手动发布 /cmd_vel (Twist)
验证:  ✓ 小车按预期移动
       ✓ /odom 里程计反馈正确
       ✓ safety_watchdog 超时正确发送零速
```

**通过标准：** `cmd_vel.linear.x = 0.5` 运行 5 秒 → 移动距离 ≈ 2.5m（允许物理引擎误差）

### 阶段 2 — 感知层验证

```
启动:  阶段 1 + LiDAR/IMU/GPS/Camera 传感器全部加载
运行:  localization_fusion(EKF) + slam_node
输入:  Gazebo 自动产生 /scan /imu /gps /camera
验证:  ✓ EKF 输出 /odom_fused 平滑稳定
       ✓ /scan 符合 Gazebo 场景布局
       ✓ SLAM 构建地图与真实场景匹配
```

**通过标准：** EKF 里程计漂移 < 5%（相对于 Gazebo 真值）

### 阶段 3 — 规划层验证

```
启动:  阶段 2 + city_street.sdf（含障碍物/路口）
运行:  behavior_tree + global_planner(A*) + local_planner(DWA)
输入:  通过 /task_goal 发布目标点
验证:  ✓ A* 规划出连通路径
       ✓ DWA 动态绕开障碍物
       ✓ 行为树状态正确转换
```

**通过标准：** 小车从起点到目标点自主导航，无碰撞

### 阶段 4 — 全栈联调

```
启动:  阶段 3 + state_machine + voice_pipeline + web_dashboard
输入:  文本命令模拟语音
验证:  ✓ 状态机正确转换（IDLE→NAVIGATING→...）
       ✓ 语音命令可中途干预路径
       ✓ web_dashboard 正确显示状态
```

### 阶段 5 （扩展） — 远程树莓派对接

PC 跑 Gazebo 仿真，树莓派 `192.168.4.12` 跑控制层 + safety_watchdog。DDS 跨机器自动发现。

---

## 5. 开发工作流

```bash
# 1. 编译
colcon build --packages-select autocar_simulation
colcon build

# 2. 仅启仿真（Gazebo + 小车模型）
source install/setup.bash
ros2 launch autocar_simulation sim_bringup.launch.py

# 3. 逐层启动（分终端）
source install/setup.bash
ros2 launch autocar_control control.launch.py sim:=true
ros2 launch autocar_perception perception.launch.py sim:=true
ros2 launch autocar_planning planning.launch.py sim:=true
ros2 launch autocar_application app.launch.py sim:=true

# 4. 发送目标点测试
ros2 topic pub /task_goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}}}"
```

### 5.1 `sim_bringup.launch.py` 逻辑

```python
def generate_launch_description():
    # 1. 启动 Gazebo
    # 2. 加载小车模型 (autocar_car.sdf)
    # 3. 加载世界 (test_obstacle.sdf / city_street.sdf)
    # 4. 启动 simulation_bridge 节点
    # 5. 设置 use_sim_time 参数
    pass
```

---

## 6. 错误处理

| 场景 | 行为 |
|------|------|
| Gazebo 未安装 | launch 时检查 `gazebo --version`，缺失则报错提示 |
| 仿真超时 | bridge 节点看门狗，仿真时钟停止 5s → 日志警告 |
| `/cmd_vel` 长时间未收到 | safety_watchdog 发零速（与真实硬件行为一致） |
| 传感器噪声参数异常 | 回退默认噪声值，日志记录 |

---

## 7. 测试策略

| 类型 | 内容 |
|------|------|
| 单元测试 | simulation_bridge 节点话题映射逻辑 |
| 集成测试 | 各层 + Gazebo 联调，ros2 bag 录制回放比对 |
| 回归测试 | 每次提交前运行全栈仿真 60s，验证不碰撞 |
