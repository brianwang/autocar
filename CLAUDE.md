# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project: Autocar

ROS2 Humble 自主小车 — 自然语音对话 + 室内外定位 + 城市路径规划。树莓派部署 (¥5000 预算)。4 层架构 16 个 ROS2 节点。

- **设计文档:** `docs/superpowers/specs/2026-06-14-autocar-system-design.md`
- **实现计划:** `docs/superpowers/plans/2026-06-14-autocar-implementation-plan.md`

## 技术栈

- **框架:** ROS2 Humble (LTS)
- **语言:** C++17 (控制层/感知层), Python 3.10+ (应用层/规划层)
- **构建:** colcon + CMake (C++) + setuptools (Python)
- **测试:** GTest (C++), pytest (Python)
- **仿真:** Gazebo Ignition

## 架构概览

```
应用层 (Python):  state_machine → voice_pipeline → web_dashboard
规划层 (C++):    behavior_tree → global_planner(A*) → local_planner(DWA)
感知层 (C++):    localization_fusion(EKF) + lidar/gps/imu/camera driver + slam_node
控制层 (C++):    velocity_controller + steering_controller → motor_driver + safety_watchdog
```

## 构建、测试、运行

```bash
# 构建全部
colcon build

# 构建单个包
colcon build --packages-select autocar_control

# 运行所有测试
colcon test --event-handlers console_direct+

# 运行单个包测试
colcon test --packages-select autocar_control --event-handlers console_direct+

# 启动全系统
source install/setup.bash
ros2 launch autocar_bringup autocar_bringup.launch.py
```

## 开发流程

**TDD 强制:** 先写测试 → 确认失败 → 写实现 → 确认通过 → 提交。

**C++ 测试模式 (GTest):**
```cpp
// test file: src/<layer>/test/test_foo.cpp
// 注册: src/<layer>/test/CMakeLists.txt → ament_add_gtest(...)
```

**Python 测试模式 (pytest):**
```bash
cd src/application && python -m pytest test/ -v
```

**自定义接口:** `src/interfaces/` 包 (`autocar_interfaces`) — 修改后需先 `colcon build --packages-select autocar_interfaces`。

## 关键话题

| 话题 | 类型 | 发布者 → 订阅者 |
|------|------|----------------|
| `/cmd_vel` | Twist | behavior_tree → velocity/steering_controller |
| `/odom` | Odometry | motor_driver → localization_fusion |
| `/odom_fused` | Odometry | localization_fusion → behavior_tree |
| `/scan` | LaserScan | lidar_driver → local_planner, slam_node |
| `/mqtt_cmd` | String (MQTT) | 远程控制台 → mqtt_command_bridge → `/cmd_vel` |
| `/voice_command` | String | voice_pipeline, mqtt_command_bridge → behavior_tree, state_machine |
| `/task_goal` | PoseStamped | state_machine → behavior_tree, global_planner |
| `/safety_state` | String | safety_watchdog → state_machine |

## 安全约束

- safety_watchdog 最高优先级 — 可绕过所有节点直接发零速
- 急停/碰撞 → 继电器切断电机供电（电路级）
- 所有 `/cmd_vel` 发布前需通过 safety_watchdog 过滤
- 电池 < 10.5V → 安全停车

## 提交前检查

- [ ] 无硬编码密钥（API key 等放 `.env`，不提交）
- [ ] 所有错误被显式处理
- [ ] 不突变数据 — 始终返回新对象
- [ ] 函数 < 50 行，文件 < 800 行，嵌套 ≤ 4 层
- [ ] 测试通过，覆盖率 ≥ 80%
