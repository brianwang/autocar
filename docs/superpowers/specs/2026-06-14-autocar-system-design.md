# Autocar — 系统架构与硬件设计文档

**日期：** 2026-06-14  
**版本：** v1.0  
**状态：** Draft

---

## 1. 项目概述

Autocar 是一台具有自然语言对话能力的室内外混合自主小车。核心能力：

- 自然语言语音对话（LLM 云端）
- 室内外实时定位与自动切换
- 城市道路级路径规划与避障
- 全自主驾驶 + 语音中途干预
- 硬件预算 ≤ ¥5000，部署为边缘核心 + 云端 LLM

---

## 2. 硬件选型

### 2.1 拓扑图

```
                      ┌──────────────────────┐
                      │   扬声器 + 麦克风       │
                      │   (USB 声卡)            │
                      └──────────┬──────────────┘
                                 │ USB
┌─────────────┐   GPIO    ┌─────┴──────────┐   USB    ┌──────────────┐
│  电机驱动     │◄─────────┤                 ├─────────►│  深度摄像头    │
│  + 编码器     │          │   树莓派 4B/5    │          │  OAK-D Lite  │
│  + 电池管理   │          │   (≥4GB RAM)    │          │  或 D435     │
└──────┬───────┘          └────┬───┬────────┘          └──────────────┘
       │                      │   │
       │ 12V                  │   │ UART
       ▼                      │   ▼
┌──────────────┐              │  ┌──────────────┐
│  Li-Po 电池  │              │  │ GPS + IMU    │
│  12V 5000mAh │              │  │ BN-880Q +    │
└──────────────┘              │  │ MPU9250      │
                              │  └──────────────┘
                              │ UART
                              ▼
                         ┌──────────────┐
                         │ 2D LiDAR     │
                         │ LD06 / X4    │
                         └──────────────┘
```

### 2.2 物料清单

| 部件 | 型号 | 价格范围 | 接口 |
|------|------|----------|------|
| 主控 | Raspberry Pi 4B/5 4GB+ | ¥400-600 | — |
| 底盘+电机+驱动 | 成品 ROS 套件 (YahBoom/Waveshare) | ¥1500-2500 | GPIO/PWM |
| 2D LiDAR | LD06 / YDLIDAR X4 | ¥400-800 | UART |
| GPS+IMU | BN-880Q + MPU9250 | ¥100-150 | UART + I2C |
| 深度摄像头 | OAK-D Lite / Intel D435 (二手) | ¥500-800 | USB3 |
| 音频 | USB 声卡 + 驻极体麦克风 + 3W 扬声器 | ¥100 | USB |
| 电源 | 12V 5000mAh Li-Po + 5V 降压模块 | ¥200-300 | — |
| **合计** | | **¥3200-5250** | |

### 2.3 功率预算

- 树莓派：~15W
- LiDAR：~3W
- 电机驱动：~20W（峰值）
- 外设（摄像头、GPS、音频）：~5W
- **总计：约 43W**
- **续航：** 60Wh / 43W ≈ 1.2 小时（5000mAh 12V）

---

## 3. 软件架构

### 3.1 分层架构图

```
┌─────────────────────────────────────────────────────────────────┐
│  应用层 (Application Layer)                                       │
│                                                                   │
│  ┌────────────┐  ┌──────────────┐  ┌──────────────┐             │
│  │ 任务编排器   │  │  语音对话管道  │  │  Web 监控面板  │             │
│  │ (state_     │  │  (voice_     │  │ (web_        │            │
│  │  machine)   │  │   pipeline)  │  │  dashboard)  │            │
│  └─────┬──────┘  └──────┬───────┘  └──────┬───────┘             │
│        │                │                  │                      │
├────────┼────────────────┼──────────────────┼──────────────────────┤
│  规划层 (Planning Layer)                        │                   │
│        │                │                  │                      │
│  ┌─────┴──────┐  ┌──────┴───────┐  ┌──────┴───────┐             │
│  │ 行为树引擎  │  │ 全局路径规划  │  │  局部避障     │              │
│  │ (behavior_ │  │ (global_     │  │  (local_     │            │
│  │  tree)     │  │  planner)    │  │  planner)    │            │
│  └─────┬──────┘  └──────┬───────┘  └──────┬───────┘             │
│        │                │                  │                      │
├────────┼────────────────┼──────────────────┼──────────────────────┤
│  感知层 (Perception Layer)                                        │
│        │                │                  │                      │
│  ┌─────┴────────────────┴──────────────────┴──────────────┐      │
│  │                    定位融合 EKF (localization_fusion)    │      │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────┐  │      │
│  │  │ GPS 解析  │  │ IMU 处理 │  │ 里程计    │  │视觉里程│  │      │
│  │  │          │  │          │  │          │  │        │  │      │
│  │  └──────────┘  └──────────┘  └──────────┘  └───────┘  │      │
│  │  ┌──────────┐                                           │      │
│  │  │ SLAM (室内)│                                          │      │
│  │  └──────────┘                                           │      │
│  └──────────────────────────────────────────────────────────┘      │
│  ┌──────────────────────────────────────────────────────────┐     │
│  │            传感器驱动层 (sensor_drivers)                    │     │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐ │     │
│  │  │ LiDAR    │  │ Camera   │  │ GPS      │  │ IMU      │ │     │
│  │  │ Driver   │  │ Driver   │  │ Driver   │  │ Driver   │ │     │
│  │  └──────────┘  └──────────┘  └──────────┘  └──────────┘ │     │
│  └──────────────────────────────────────────────────────────┘      │
├─────────────────────────────────────────────────────────────────── │
│  控制层 (Control Layer)                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐           │
│  │ 速度控制器    │  │ 转向控制器    │  │ 安全守护       │           │
│  │ (velocity_   │  │ (steering_   │  │ (safety_     │          │
│  │  controller) │  │  controller) │  │  watchdog)   │          │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘           │
│         │                 │                  │                    │
│  ┌──────┴─────────────────┴──────────────────┴─────────────────┐  │
│  │              电机驱动接口 (motor_driver)                      │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │  │
│  │  │ PWM 输出      │  │ 编码器读取     │  │ 急停/限位     │      │  │
│  │  └──────────────┘  └──────────────┘  └──────────────┘      │  │
│  └─────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────────────┘
```

### 3.2 技术栈

- **框架：** ROS2 Humble (LTS)
- **语言：** C++17（感知层、控制层）、Python 3.10+（应用层、规划层）
- **构建：** colcon + CMake + setuptools
- **仿真：** Gazebo Ignition
- **测试：** GTest (C++), pytest (Python), rosbag replay
- **云端 API：** DeepSeek/通义千问（LLM）、Whisper（ASR）、Edge-TTS

### 3.3 ROS2 节点清单

**应用层 (3):**

| 节点 | 语言 | 职责 |
|------|------|------|
| state_machine | Python | 任务编排、模式切换 |
| voice_pipeline | Python | 唤醒→ASR→LLM→TTS→指令解析 |
| web_dashboard | Python | ROS2 web bridge 显示状态/地图 |

**规划层 (3):**

| 节点 | 语言 | 职责 |
|------|------|------|
| behavior_tree | C++ | 行为树引擎，消费指令和规划结果 |
| global_planner | C++ | A*/Dijkstra 全局路径规划 |
| local_planner | C++ | DWA/TEB 局部避障 |

**感知层 (6):**

| 节点 | 语言 | 职责 |
|------|------|------|
| localization_fusion | C++ | EKF 融合，室内外切换 |
| lidar_driver | C++ | LiDAR 驱动 |
| camera_driver | C++ | 深度/RGB 摄像头驱动 |
| gps_driver | C++ | GPS NMEA 解析 |
| imu_driver | C++ | IMU 数据发布 |
| slam_node | C++ | Cartographer (室内) |

**控制层 (4):**

| 节点 | 语言 | 职责 |
|------|------|------|
| velocity_controller | C++ | PID 线速度控制 |
| steering_controller | C++ | PID 转向控制 |
| safety_watchdog | C++ | 急停、电池、碰撞、看门狗 |
| motor_driver | C++ | 硬件 PWM/编码器抽象 |

---

## 4. 通信与数据流

### 4.1 核心话题

**感知发布：**

| 话题 | 类型 | 发布者 → 订阅者 |
|------|------|----------------|
| `/scan` | LaserScan | lidar_driver → slam_node, local_planner |
| `/image_raw` | Image | camera_driver → slam_node |
| `/depth` | Image | camera_driver → local_planner |
| `/fix` | NavSatFix | gps_driver → localization_fusion |
| `/imu` | Imu | imu_driver → localization_fusion |
| `/odom` | Odometry | motor_driver → localization_fusion |

**导航与指令：**

| 话题 | 类型 | 发布者 → 订阅者 |
|------|------|----------------|
| `/voice_command` | String | voice_pipeline → behavior_tree |
| `/task_goal` | PoseStamped | state_machine → behavior_tree |
| `/cmd_vel` | Twist | behavior_tree → velocity/steering_controller |
| `/global_path` | Path | global_planner → behavior_tree |
| `/local_path` | Path | local_planner → behavior_tree |
| `/odom_fused` | Odometry | localization_fusion → behavior_tree, slam_node |

**安全：**

| 话题 | 类型 | 发布者 → 订阅者 |
|------|------|----------------|
| `/battery_voltage` | Float32 | motor_driver → safety_watchdog |
| `/e_stop` | Bool | GPIO 中断 → safety_watchdog |
| `/safety_state` | String | safety_watchdog → state_machine |

### 4.2 服务

| 服务 | 提供者 | 消费者 |
|------|--------|--------|
| `/plan_request` | global_planner | behavior_tree |
| `/query_local_plan` | local_planner | behavior_tree |

### 4.3 安全回路

safety_watchdog 有最高优先级：
1. 订阅 `/cmd_vel`，异常指令直接屏蔽
2. 电池 ≤ 10.5V → 安全停车
3. 硬件急停按钮 → GPIO 中断 → 继电器切断电机供电
4. 心跳超时 500ms → 安全停车
5. 任一安全条件触发 → 绕过所有其他节点，直接发零速

### 4.4 发声管道延迟预算

```
麦克风 → VAD(本地 silero) → 唤醒词(Porcupine) → ASR(云端 Whisper) 
       → LLM(云端流式) → 意图解析(BERT 本地) → /voice_command → behavior_tree
       → 执行 → TTS(云端流式) → 扬声器
```

| 环节 | 位置 | 延迟 |
|------|------|------|
| 唤醒词 | 本地 | < 100ms |
| ASR 首字节 | 云端 | < 300ms |
| LLM 首 token | 云端 | < 800ms |
| TTS 首字节 | 云端 | < 500ms |
| **语音输入→开始播报** | | **约 1.5-2s** |

---

## 5. 任务编排与行为树

### 5.1 系统状态机

```
                    ┌──────────┐
                    │  BOOTUP  │
                    └────┬─────┘
                         │ 传感器自检通过
                    ┌────▼─────┐
                    │   IDLE   │◄──────────────────┐
                    └────┬─────┘                    │
                         │ 语音"出发"/"去XX"        │
                    ┌────▼─────┐   急停/异常        │
                    │ AUTONOMY ├──────────┐         │
                    └──┬───┬───┘          │         │
        语音"停下"     │   │  语音干预方向    │    ┌───▼──┐
      ┌───────────────┘   └──────────┐    │    │ESTOP │
      ▼                              ▼    │    └───┬──┘
┌──────────┐                   ┌──────────┐    │
│  PAUSED  ├───────────────────►│  RESUME  │    │
└─────┬────┘                   └────┬─────┘    │
      │                             │          │
      └─────────────────────────────┘          │
              语音"继续" / "好了"               │
                                               │
      ┌────────────────────────────────────────┘
      │ 安全恢复
      ▼
   (回到 IDLE)
```

5 种模式，严格受 safety_watchdog 约束。

### 5.2 行为树结构

```
ReactiveSequence ["去菜鸟驿站" 任务]
├─ Condition (safety_ok?)         ← 每 tick 检查
├─ Fallback [定位健壮性]
│  ├─ Action (室外 GPS 定位)
│  └─ Action (室内 SLAM 定位)
├─ Fallback [路径获取]
│  ├─ Action (调用 global_planner)
│  └─ Action ("无法规划" → 语音反馈)
├─ Sequence [路径跟踪循环, 1 Hz 重规划]
│  ├─ Action (调用 local_planner)
│  ├─ Action (发布 /cmd_vel)
│  └─ Condition (未到达 && 无动态障碍 && safety_ok?)
└─ Action (到达 → TTS 播报)
```

### 5.3 语音意图映射

| 用户说 | 意图 | 参数 |
|--------|------|------|
| "去 3 号楼" | `NAVIGATE` | target=3号楼 |
| "停一下" | `PAUSE` | — |
| "别走左边，走右边" | `OVERRIDE_PLAN` | constraint=prefer_right |
| "前面有人，绕一下" | `REPLAN` | reason=pedestrian |
| "回车库" | `NAVIGATE` | target=home |
| "还有多远" | `QUERY` | topic=remaining_distance |

---

## 6. 语音管道

### 6.1 组件选型

| 环节 | 推荐方案 | 备选 | 月费估算 |
|------|---------|------|---------|
| VAD | Silero VAD (本地) | WebRTC VAD | — |
| 唤醒词 | Porcupine (本地, 免费 3 词) | Snowboy | — |
| ASR | 阿里云一句话识别 / 智谱 Whisper | Azure STT | ¥50-200 |
| LLM | DeepSeek / 通义千问 | GPT-4o-mini | ¥100-500 |
| TTS | Edge-TTS (免费) / 火山引擎 | Azure TTS | ¥0-50 |
| 意图解析 | BERT tiny (本地) / 规则引擎 | — | — |

### 6.2 LLM 超时与降级

- 请求超时 5s → 重试 2 次 → 仍失败播报"语音服务暂时不可用"
- 流式 token 间隔 >3s → 断连重试
- 返回内容解析失败 → 规则引擎兜底 → 仅识别核心意图词
- 所有云端 API 需网络，离线时小车只能手动/遥控

---

## 7. 室内外定位切换

### 7.1 切换策略

| 模式 | 传感器 | 条件 |
|------|--------|------|
| 室外 | GPS + IMU + 轮式里程计 → EKF | 卫星数 ≥ 6, HDOP < 2 |
| 室内 | Cartographer SLAM (LiDAR + IMU + 里程计) | 卫星数 < 4 持续 3s |
| 过渡 | 锚点 + 里程计推演 + 平滑插值 | 切换中 |

### 7.2 降级处理

- GPS 丢失 → 最后 GPS + 里程计推演 → 设置锚点
- SLAM 未锁定 → 降速至 0.2m/s → 10s 超时安全停车
- HDOP > 5 → 禁止切换到自主模式
- 切换时位姿平滑插值，不跳变

---

## 8. 安全体系

### 8.1 安全等级

| 等级 | 触发条件 | 响应 | 恢复 |
|------|---------|------|------|
| **L1 致命** | 急停按下、碰撞检测、电池临界、CPU 过热 | 继电器切断电机供电 | 人工复位 |
| **L2 严重** | 电池低压、传感器连续丢失 >5s、通信中断 >3s | 安全停车 | 超时降级或人工 |
| **L3 警告** | GPS 仅丢失（切换中）、电机温度偏高、WiFi 弱 | 降速、播报 | 自动恢复 |

### 8.2 safety_watchdog 关键逻辑

- 独立硬件看门狗定时器
- `/cmd_vel` 指令超时 >200ms → 零速
- 主循环心跳超时 >500ms → 零速
- 电池 < 10.5V → 安全停车
- 急停/碰撞 → 继电器直接切断（电路级保护）

---

## 9. 测试策略

### 9.1 测试金字塔

| 层级 | 工具 | 范围 |
|------|------|------|
| 单元测试 | GTest (C++) + pytest (Python) | EKF, A*, PID, 行为树, 意图解析 |
| 集成测试 | ROS2 launch_test + rosbag replay | 每个话题/Service 契约 |
| 仿真测试 | Gazebo Ignition | 室内外切换, 避障, 路径跟踪 |
| 实车测试 | rosbag 录制 + 回放对比 | 5 条测试路线 |

### 9.2 仿真场景

- 室内世界：50m × 30m 走廊+房间，家具障碍物
- 室外世界：200m × 200m 城市街区，模拟 GPS 噪声
- 过渡区：建筑出入口，GPS 逐渐丢失/恢复

### 9.3 联调阶段

```
Phase 1: 电控联调 → 电机响应、PID 整定、里程计精度
Phase 2: 传感器联调 → 各驱动数据发布验证、EKF 收敛
Phase 3: 室内自主 → SLAM 建图、路径规划、避障
Phase 4: 室外自主 → GPS 路径跟踪、交叉路口
Phase 5: 语音集成 → 自然对话、指令解析、模式切换
Phase 6: 全系统联调 → 室内外切换、异常场景、长跑
```

---

## 10. 项目结构

```
autocar/
├── src/
│   ├── application/          # 应用层 (Python)
│   │   ├── state_machine/
│   │   ├── voice_pipeline/
│   │   └── web_dashboard/
│   ├── planning/             # 规划层 (C++)
│   │   ├── behavior_tree/
│   │   ├── global_planner/
│   │   └── local_planner/
│   ├── perception/           # 感知层 (C++)
│   │   ├── localization_fusion/
│   │   ├── lidar_driver/
│   │   ├── camera_driver/
│   │   ├── gps_driver/
│   │   ├── imu_driver/
│   │   └── slam_node/
│   ├── control/              # 控制层 (C++)
│   │   ├── velocity_controller/
│   │   ├── steering_controller/
│   │   ├── safety_watchdog/
│   │   └── motor_driver/
│   └── interfaces/           # 共享接口定义
│       ├── msg/
│       └── srv/
├── config/                   # ROS2 参数与启动文件
│   ├── launch/
│   └── params/
├── simulation/               # Gazebo 仿真
│   ├── worlds/
│   └── models/
├── test/
│   ├── unit/
│   └── integration/
├── docs/
│   └── superpowers/
│       └── specs/
└── README.md
```

---

## 11. 服务供应商

| 服务 | 推荐 | 月费估算 |
|------|------|---------|
| ASR | 阿里云一句话识别 | ¥50-200 |
| LLM | DeepSeek / 通义千问 | ¥100-500 |
| TTS | Edge-TTS (免费) | ¥0 |
| 唤醒词 | Porcupine (本地) | 免费 |
| VAD | Silero VAD (本地) | 免费 |

---

## 12. 风险与缓解

| 风险 | 影响 | 缓解 |
|------|------|------|
| 树莓派算力不足跑实时 SLAM | 定位延迟 | 用 Cartographer 轻量配置，必要时换 Jetson |
| 室内外切换跳变 | 定位错误 | 锚点 + 平滑插值 + 切换保护时间 |
| 云端 LLM 延迟过高 | 对话体验差 | 流式返回 + 本地 TTS 预加载 + 降级规则引擎 |
| WiFi 断开 | 语音不可用 | 离线降级为手动/遥控 + 本地规则指令 |
| LiDAR 室内建图质量差 | 路径规划不准 | 深度摄像头辅助 + 更保守的速度参数 |

---

## 13. 下一步

此设计涵盖整体架构。实现将按以下顺序进行：

1. 硬件采购与组装
2. ROS2 骨架搭建 + 电机控制联调
3. 传感器驱动与定位融合
4. 路径规划与自主驾驶
5. 语音管道集成
6. 全系统联调与测试
