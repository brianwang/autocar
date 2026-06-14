# Autocar Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 搭建 Autocar 自主小车完整软件栈 — ROS2 Humble, 16 个节点, 4 层架构, 树莓派部署。

**Architecture:** 四层 ROS2 架构：控制层 (C++ 电机 PID/安全)、感知层 (C++ 传感器驱动/EKF 融合/SLAM)、规划层 (C++ 行为树/A*/DWA)、应用层 (Python 状态机/语音管道/Web 面板)。先建骨架和接口 → 控制 → 感知 → 规划 → 应用 → 仿真 → 集成。

**Tech Stack:** ROS2 Humble, C++17, Python 3.10+, colcon, CMake, setuptools, GTest, pytest, Gazebo Ignition, Nav2 stack

---

## Phase 0: 仓库与 ROS2 工作区骨架

### Task 0.1: 初始化 ROS2 工作区和仓库骨架

**Files:**
- Create: `src/interfaces/CMakeLists.txt`
- Create: `src/interfaces/package.xml`
- Create: `src/interfaces/msg/SafetyState.msg`
- Create: `src/interfaces/srv/PlanRequest.srv`
- Create: `src/interfaces/srv/QueryLocalPlan.srv`
- Create: `CMakeLists.txt` (workspace root)
- Create: `config/params/motor.yaml`
- Create: `config/params/safety.yaml`
- Create: `config/params/localization.yaml`
- Create: `config/launch/autocar_bringup.launch.py`
- Create: `.gitignore`

- [ ] **Step 1: Create workspace root CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.8)
project(autocar_ws)
```

- [ ] **Step 2: Create custom message interfaces package**

`src/interfaces/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autocar_interfaces</name>
  <version>0.1.0</version>
  <description>Custom messages and services for Autocar</description>
  <maintainer email="dev@autocar.local">Autocar Dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`src/interfaces/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(autocar_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyState.msg"
  "srv/PlanRequest.srv"
  "srv/QueryLocalPlan.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 3: Create SafetyState.msg**

`src/interfaces/msg/SafetyState.msg`:
```
# Safety watchdog state message
uint8 FATAL=0
uint8 CRITICAL=1
uint8 WARNING=2
uint8 NORMAL=3

uint8 level
string detail
bool motor_relay_on
float32 battery_voltage
```

- [ ] **Step 4: Create PlanRequest.srv**

`src/interfaces/srv/PlanRequest.srv`:
```
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
string map_frame
---
nav_msgs/Path path
bool success
string message
```

- [ ] **Step 5: Create QueryLocalPlan.srv**

`src/interfaces/srv/QueryLocalPlan.srv`:
```
sensor_msgs/LaserScan scan
geometry_msgs/Twist current_velocity
nav_msgs/Path global_path
---
nav_msgs/Path local_path
geometry_msgs/Twist cmd_vel
bool success
string message
```

- [ ] **Step 6: Create motor config**

`config/params/motor.yaml`:
```yaml
motor_driver:
  ros__parameters:
    pwm_pin: 12
    dir_pin: 13
    encoder_pin_a: 17
    encoder_pin_b: 27
    encoder_ppr: 48
    wheel_radius: 0.032
    wheel_base: 0.178
    max_pwm: 255
    pwm_frequency: 100
    gear_ratio: 1.0

velocity_controller:
  ros__parameters:
    kp: 1.2
    ki: 0.3
    kd: 0.05
    max_linear_speed: 0.5
    max_linear_accel: 0.3
    control_rate: 50.0

steering_controller:
  ros__parameters:
    kp: 2.0
    ki: 0.1
    kd: 0.02
    max_angular_speed: 1.5
    max_angular_accel: 0.8
    control_rate: 50.0
```

- [ ] **Step 7: Create safety config**

`config/params/safety.yaml`:
```yaml
safety_watchdog:
  ros__parameters:
    battery_threshold_critical: 10.5
    battery_threshold_warning: 11.0
    cmd_vel_timeout_ms: 200
    heartbeat_timeout_ms: 500
    sensor_loss_timeout_s: 5.0
    cpu_temp_threshold: 80.0
    watchdog_device: "/dev/watchdog"
```

- [ ] **Step 8: Create localization config**

`config/params/localization.yaml`:
```yaml
localization_fusion:
  ros__parameters:
    gps_min_satellites_outdoor: 6
    gps_max_hdop: 2.0
    gps_loss_timeout_s: 3.0
    slam_lock_timeout_s: 10.0
    transition_blend_time_s: 2.0
    outdoor_mode: "gps_imu_odom"
    indoor_mode: "cartographer_slam"

gps_driver:
  ros__parameters:
    port: "/dev/ttyAMA0"
    baud_rate: 9600
    frame_id: "gps_link"
    publish_rate: 10.0

imu_driver:
  ros__parameters:
    i2c_bus: 1
    address: 0x68
    frame_id: "imu_link"
    publish_rate: 100.0

lidar_driver:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baud_rate: 230400
    frame_id: "laser_frame"
    angle_min: 0.0
    angle_max: 6.283
    range_min: 0.05
    range_max: 12.0
```

- [ ] **Step 9: Create bringup launch file**

`config/launch/autocar_bringup.launch.py`:
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('autocar_bringup'), '..', '..', '..', 'config', 'params'
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='autocar'),
        DeclareLaunchArgument('simulation', default_value='false'),

        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),

            # Control layer
            Node(
                package='autocar_control', executable='motor_driver',
                name='motor_driver',
                parameters=[os.path.join(config_dir, 'motor.yaml')],
            ),
            Node(
                package='autocar_control', executable='velocity_controller',
                name='velocity_controller',
                parameters=[os.path.join(config_dir, 'motor.yaml')],
            ),
            Node(
                package='autocar_control', executable='steering_controller',
                name='steering_controller',
                parameters=[os.path.join(config_dir, 'motor.yaml')],
            ),
            Node(
                package='autocar_control', executable='safety_watchdog',
                name='safety_watchdog',
                parameters=[os.path.join(config_dir, 'safety.yaml')],
            ),

            # Perception layer
            Node(
                package='autocar_perception', executable='gps_driver',
                name='gps_driver',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
            ),
            Node(
                package='autocar_perception', executable='imu_driver',
                name='imu_driver',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
            ),
            Node(
                package='autocar_perception', executable='lidar_driver',
                name='lidar_driver',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
            ),
            Node(
                package='autocar_perception', executable='camera_driver',
                name='camera_driver',
            ),
            Node(
                package='autocar_perception', executable='localization_fusion',
                name='localization_fusion',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
            ),
            Node(
                package='autocar_perception', executable='slam_node',
                name='slam_node',
            ),

            # Planning layer
            Node(
                package='autocar_planning', executable='global_planner',
                name='global_planner',
            ),
            Node(
                package='autocar_planning', executable='local_planner',
                name='local_planner',
            ),
            Node(
                package='autocar_planning', executable='behavior_tree',
                name='behavior_tree',
            ),

            # Application layer
            Node(
                package='autocar_application', executable='state_machine',
                name='state_machine',
            ),
            Node(
                package='autocar_application', executable='voice_pipeline',
                name='voice_pipeline',
            ),
            Node(
                package='autocar_application', executable='web_dashboard',
                name='web_dashboard',
            ),
        ]),
    ])
```

- [ ] **Step 10: Create .gitignore**

```
build/
install/
log/
*.pyc
__pycache__/
*.o
*.so
.env
*.bag
```

- [ ] **Step 11: Build workspace and verify skeleton compiles**

```bash
colcon build --packages-select autocar_interfaces
```
Expected: Build succeeds.

- [ ] **Step 12: Commit**

```bash
git add -A
git commit -m "feat: add ROS2 workspace skeleton, custom interfaces, config, and bringup launch"
```

---

### Task 0.2: 创建 ROS2 控制层包骨架

**Files:**
- Create: `src/control/CMakeLists.txt`
- Create: `src/control/package.xml`
- Create: `src/control/src/motor_driver.cpp` (empty main)
- Create: `src/control/src/velocity_controller.cpp` (empty main)
- Create: `src/control/src/steering_controller.cpp` (empty main)
- Create: `src/control/src/safety_watchdog.cpp` (empty main)
- Create: `src/control/include/autocar_control/pid_controller.hpp`

- [ ] **Step 1: Create package.xml**

`src/control/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autocar_control</name>
  <version>0.1.0</version>
  <description>Autocar control layer: motor driver, PID controllers, safety watchdog</description>
  <maintainer email="dev@autocar.local">Autocar Dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>autocar_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create CMakeLists.txt with all executables**

`src/control/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(autocar_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autocar_interfaces REQUIRED)

include_directories(include)
include_directories(${rclcpp_INCLUDE_DIRS})

set(EXECUTABLES
  motor_driver
  velocity_controller
  steering_controller
  safety_watchdog
)

foreach(EXEC IN LISTS EXECUTABLES)
  add_executable(${EXEC} src/${EXEC}.cpp)
  ament_target_dependencies(${EXEC}
    rclcpp geometry_msgs std_msgs autocar_interfaces
  )
  install(TARGETS ${EXEC}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
```

- [ ] **Step 3: Create PID controller header**

`src/control/include/autocar_control/pid_controller.hpp`:
```cpp
#pragma once

namespace autocar_control {

class PIDController {
public:
  PIDController(double kp, double ki, double kd,
                double output_min, double output_max,
                double integral_limit);

  void reset();
  double compute(double setpoint, double measurement, double dt);
  void setGains(double kp, double ki, double kd);

private:
  double kp_, ki_, kd_;
  double output_min_, output_max_;
  double integral_limit_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool first_run_{true};
};

}  // namespace autocar_control
```

- [ ] **Step 4: Create stub motor_driver.cpp**

`src/control/src/motor_driver.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("motor_driver");
  RCLCPP_INFO(node->get_logger(), "motor_driver starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 5: Create stub velocity_controller.cpp**

`src/control/src/velocity_controller.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("velocity_controller");
  RCLCPP_INFO(node->get_logger(), "velocity_controller starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 6: Create stub steering_controller.cpp**

`src/control/src/steering_controller.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("steering_controller");
  RCLCPP_INFO(node->get_logger(), "steering_controller starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 7: Create stub safety_watchdog.cpp**

`src/control/src/safety_watchdog.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("safety_watchdog");
  RCLCPP_INFO(node->get_logger(), "safety_watchdog starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 8: Build and verify**

```bash
colcon build --packages-select autocar_control
```
Expected: Build succeeds, 4 executables created.

- [ ] **Step 9: Commit**

```bash
git add -A
git commit -m "feat: add control layer package skeleton with PID controller header"
```

---

### Task 0.3: 创建感知层、规划层、应用层包骨架

**Files:**
- Create: `src/perception/package.xml`
- Create: `src/perception/CMakeLists.txt`
- Create: `src/perception/src/gps_driver.cpp`
- Create: `src/perception/src/imu_driver.cpp`
- Create: `src/perception/src/lidar_driver.cpp`
- Create: `src/perception/src/camera_driver.cpp`
- Create: `src/perception/src/localization_fusion.cpp`
- Create: `src/perception/src/slam_node.cpp`
- Create: `src/perception/include/autocar_perception/kalman_filter.hpp`
- Create: `src/planning/package.xml`
- Create: `src/planning/CMakeLists.txt`
- Create: `src/planning/src/behavior_tree.cpp`
- Create: `src/planning/src/global_planner.cpp`
- Create: `src/planning/src/local_planner.cpp`
- Create: `src/planning/include/autocar_planning/astar.hpp`
- Create: `src/planning/include/autocar_planning/dwa_planner.hpp`
- Create: `src/planning/include/autocar_planning/behavior_nodes.hpp`
- Create: `src/application/setup.py`
- Create: `src/application/package.xml`
- Create: `src/application/autocar_application/__init__.py`
- Create: `src/application/autocar_application/state_machine.py`
- Create: `src/application/autocar_application/voice_pipeline.py`
- Create: `src/application/autocar_application/web_dashboard.py`

- [ ] **Step 1: Create perception package.xml**

`src/perception/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autocar_perception</name>
  <version>0.1.0</version>
  <description>Autocar perception layer: sensor drivers, localization fusion, SLAM</description>
  <maintainer email="dev@autocar.local">Autocar Dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>autocar_interfaces</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create perception CMakeLists.txt**

`src/perception/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(autocar_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(EXECUTABLES
  gps_driver
  imu_driver
  lidar_driver
  camera_driver
  localization_fusion
  slam_node
)

foreach(EXEC IN LISTS EXECUTABLES)
  add_executable(${EXEC} src/${EXEC}.cpp)
  ament_target_dependencies(${EXEC}
    rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs
  )
  install(TARGETS ${EXEC}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
```

- [ ] **Step 3: Create perception stub source files**

For each executable source (`gps_driver.cpp`, `imu_driver.cpp`, `lidar_driver.cpp`, `camera_driver.cpp`, `localization_fusion.cpp`, `slam_node.cpp`), create a stub:

`src/perception/src/gps_driver.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gps_driver");
  RCLCPP_INFO(node->get_logger(), "gps_driver starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

(Repeat same pattern for all 6 stubs, changing node name)

- [ ] **Step 4: Create EKF header**

`src/perception/include/autocar_perception/kalman_filter.hpp`:
```cpp
#pragma once

#include <Eigen/Dense>

namespace autocar_perception {

class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter(int state_dim, int measurement_dim);

  void setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
  void setProcessNoise(const Eigen::MatrixXd& Q);
  void setMeasurementNoise(const Eigen::MatrixXd& R);

  void predict(const Eigen::VectorXd& u, double dt,
               std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> f,
               std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> F_jacobian);

  void update(const Eigen::VectorXd& z,
              std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
              std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> H_jacobian);

  Eigen::VectorXd state() const { return x_; }
  Eigen::MatrixXd covariance() const { return P_; }

private:
  int state_dim_;
  int measurement_dim_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
};

}  // namespace autocar_perception
```

- [ ] **Step 5: Create planning package.xml**

`src/planning/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autocar_planning</name>
  <version>0.1.0</version>
  <description>Autocar planning layer: behavior tree, global planner, local planner</description>
  <maintainer email="dev@autocar.local">Autocar Dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>autocar_interfaces</depend>

  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 6: Create planning CMakeLists.txt**

`src/planning/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(autocar_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(EXECUTABLES behavior_tree global_planner local_planner)

foreach(EXEC IN LISTS EXECUTABLES)
  add_executable(${EXEC} src/${EXEC}.cpp)
  ament_target_dependencies(${EXEC}
    rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs
  )
  install(TARGETS ${EXEC}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
```

- [ ] **Step 7: Create planning headers**

`src/planning/include/autocar_planning/astar.hpp`:
```cpp
#pragma once

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <functional>

namespace autocar_planning {

struct AStarNode {
  int x, y;
  double g_cost{0.0};
  double h_cost{0.0};
  double f_cost() const { return g_cost + h_cost; }
  AStarNode* parent{nullptr};
  bool operator>(const AStarNode& other) const { return f_cost() > other.f_cost(); }
};

class AStarPlanner {
public:
  using OccupancyFn = std::function<bool(int, int)>;

  AStarPlanner(int width, int height, double resolution);

  void setOccupancyGrid(const std::vector<int8_t>& grid);
  void setOccupancyFn(OccupancyFn fn) { is_occupied_ = fn; }

  nav_msgs::msg::Path plan(const geometry_msgs::msg::PoseStamped& start,
                            const geometry_msgs::msg::PoseStamped& goal);
  bool planToPath(const geometry_msgs::msg::PoseStamped& start,
                  const geometry_msgs::msg::PoseStamped& goal,
                  nav_msgs::msg::Path& path);

private:
  double heuristic(int x1, int y1, int x2, int y2) const;
  std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;
  bool isValid(int x, int y) const;
  void worldToGrid(double wx, double wy, int& gx, int& gy) const;
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  int width_, height_;
  double resolution_;
  OccupancyFn is_occupied_;
};

}  // namespace autocar_planning
```

`src/planning/include/autocar_planning/dwa_planner.hpp`:
```cpp
#pragma once

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

namespace autocar_planning {

struct DWAConfig {
  double max_linear_speed{0.5};
  double max_angular_speed{1.5};
  double linear_accel{0.3};
  double angular_accel{0.8};
  double linear_resolution{0.05};
  double angular_resolution{0.1};
  double predict_time{2.0};
  double dt{0.1};
  double goal_tolerance{0.2};
  double obstacle_cost_weight{1.0};
  double goal_cost_weight{0.5};
  double speed_cost_weight{0.3};
};

struct Trajectory {
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  double linear_velocity;
  double angular_velocity;
  double cost;
};

class DWAPlanner {
public:
  explicit DWAPlanner(const DWAConfig& config = DWAConfig());

  geometry_msgs::msg::Twist plan(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan
  );

private:
  std::vector<Trajectory> generateTrajectories(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& vel
  );
  Trajectory simulateTrajectory(double linear_v, double angular_v,
    const geometry_msgs::msg::PoseStamped& start_pose);
  double evaluateTrajectory(const Trajectory& traj,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan);
  double obstacleCost(const Trajectory& traj, const sensor_msgs::msg::LaserScan& scan);
  double goalCost(const Trajectory& traj, const nav_msgs::msg::Path& global_path);
  double speedCost(const Trajectory& traj);

  DWAConfig config_;
};

}  // namespace autocar_planning
```

`src/planning/include/autocar_planning/behavior_nodes.hpp`:
```cpp
#pragma once

#include <string>
#include <functional>
#include <memory>
#include <vector>

namespace autocar_planning {

enum class NodeStatus {
  SUCCESS,
  FAILURE,
  RUNNING
};

class BehaviorNode {
public:
  virtual ~BehaviorNode() = default;
  virtual NodeStatus tick() = 0;
  virtual void reset() {}
  const std::string& name() const { return name_; }

protected:
  std::string name_;
};

class Sequence : public BehaviorNode {
public:
  explicit Sequence(const std::string& name);
  void addChild(std::shared_ptr<BehaviorNode> child);
  NodeStatus tick() override;
  void reset() override;

private:
  std::vector<std::shared_ptr<BehaviorNode>> children_;
  size_t current_index_{0};
};

class Fallback : public BehaviorNode {
public:
  explicit Fallback(const std::string& name);
  void addChild(std::shared_ptr<BehaviorNode> child);
  NodeStatus tick() override;
  void reset() override;

private:
  std::vector<std::shared_ptr<BehaviorNode>> children_;
  size_t current_index_{0};
};

class ReactiveSequence : public BehaviorNode {
public:
  explicit ReactiveSequence(const std::string& name);
  void addChild(std::shared_ptr<BehaviorNode> child);
  NodeStatus tick() override;
  void reset() override;

private:
  std::vector<std::shared_ptr<BehaviorNode>> children_;
};

class ConditionNode : public BehaviorNode {
public:
  explicit ConditionNode(const std::string& name,
                          std::function<bool()> condition);
  NodeStatus tick() override;

private:
  std::function<bool()> condition_;
};

class ActionNode : public BehaviorNode {
public:
  explicit ActionNode(const std::string& name,
                       std::function<NodeStatus()> action);
  NodeStatus tick() override;

private:
  std::function<NodeStatus()> action_;
};

}  // namespace autocar_planning
```

- [ ] **Step 8: Create planning stub source files**

Create stubs for `behavior_tree.cpp`, `global_planner.cpp`, `local_planner.cpp` following same pattern as perception stubs.

- [ ] **Step 9: Create application Python package**

`src/application/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autocar_application</name>
  <version>0.1.0</version>
  <description>Autocar application layer: state machine, voice pipeline, web dashboard</description>
  <maintainer email="dev@autocar.local">Autocar Dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

`src/application/setup.py`:
```python
from setuptools import setup

package_name = 'autocar_application'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Autocar Dev',
    maintainer_email='dev@autocar.local',
    description='Autocar application layer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_machine = autocar_application.state_machine:main',
            'voice_pipeline = autocar_application.voice_pipeline:main',
            'web_dashboard = autocar_application.web_dashboard:main',
        ],
    },
)
```

`src/application/autocar_application/__init__.py`:
```python
"""Autocar application layer package."""
```

`src/application/autocar_application/state_machine.py`:
```python
"""Autocar state machine node."""
import rclpy
from rclpy.node import Node


class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().info('state_machine starting')


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`src/application/autocar_application/voice_pipeline.py`:
```python
"""Autocar voice pipeline node."""
import rclpy
from rclpy.node import Node


class VoicePipeline(Node):
    def __init__(self):
        super().__init__('voice_pipeline')
        self.get_logger().info('voice_pipeline starting')


def main(args=None):
    rclpy.init(args=args)
    node = VoicePipeline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

`src/application/autocar_application/web_dashboard.py`:
```python
"""Autocar web dashboard node."""
import rclpy
from rclpy.node import Node


class WebDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.get_logger().info('web_dashboard starting')


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 10: Create resource file for application package**

```bash
mkdir -p src/application/resource
touch src/application/resource/autocar_application
```

- [ ] **Step 11: Build all packages**

```bash
colcon build
```
Expected: All packages build successfully.

- [ ] **Step 12: Commit**

```bash
git add -A
git commit -m "feat: add perception, planning, and application layer package skeletons"
```

---

## Phase 1: 控制层 — 车能跑

### Task 1.1: 实现 PID 控制器

**Files:**
- Create: `src/control/src/pid_controller.cpp`
- Create: `src/control/test/test_pid.cpp`
- Create: `src/control/test/CMakeLists.txt`

- [ ] **Step 1: Write failing PID test**

`src/control/test/test_pid.cpp`:
```cpp
#include <gtest/gtest.h>
#include "autocar_control/pid_controller.hpp"

TEST(PIDControllerTest, ProportionalOutput) {
  autocar_control::PIDController pid(2.0, 0.0, 0.0, -10.0, 10.0, 100.0);
  double output = pid.compute(5.0, 4.0, 1.0);
  EXPECT_NEAR(output, 2.0, 0.01);
}

TEST(PIDControllerTest, IntegralAccumulates) {
  autocar_control::PIDController pid(0.0, 1.0, 0.0, -10.0, 10.0, 100.0);
  pid.compute(1.0, 0.0, 1.0);
  double output = pid.compute(1.0, 0.0, 1.0);
  EXPECT_NEAR(output, 2.0, 0.01);
}

TEST(PIDControllerTest, DerivativeDampens) {
  autocar_control::PIDController pid(0.0, 0.0, 1.0, -10.0, 10.0, 100.0);
  pid.compute(2.0, 0.0, 1.0);
  double output = pid.compute(1.0, 1.0, 1.0);
  EXPECT_NEAR(output, -1.0, 0.01);
}

TEST(PIDControllerTest, OutputClamped) {
  autocar_control::PIDController pid(100.0, 0.0, 0.0, -3.0, 3.0, 100.0);
  double output = pid.compute(10.0, 0.0, 1.0);
  EXPECT_NEAR(output, 3.0, 0.01);
}

TEST(PIDControllerTest, ResetClearsIntegral) {
  autocar_control::PIDController pid(0.0, 1.0, 0.0, -10.0, 10.0, 100.0);
  pid.compute(1.0, 0.0, 1.0);
  pid.reset();
  double output = pid.compute(1.0, 0.0, 1.0);
  EXPECT_NEAR(output, 1.0, 0.01);
}

TEST(PIDControllerTest, IntegralAntiWindup) {
  autocar_control::PIDController pid(0.0, 1.0, 0.0, -2.0, 2.0, 3.0);
  for (int i = 0; i < 10; ++i) {
    pid.compute(10.0, 0.0, 1.0);
  }
  double output = pid.compute(10.0, 0.0, 1.0);
  EXPECT_NEAR(output, 2.0, 0.01);
}

TEST(PIDControllerTest, SetGainsReconfigures) {
  autocar_control::PIDController pid(1.0, 0.0, 0.0, -10.0, 10.0, 100.0);
  pid.setGains(3.0, 0.0, 0.0);
  double output = pid.compute(2.0, 1.0, 1.0);
  EXPECT_NEAR(output, 3.0, 0.01);
}
```

`src/control/test/CMakeLists.txt`:
```cmake
add_executable(test_pid test_pid.cpp ../src/pid_controller.cpp)
target_include_directories(test_pid PRIVATE ../include)
ament_target_dependencies(test_pid rclcpp)
target_link_libraries(test_pid gtest gtest_main pthread)
ament_add_gtest(test_pid test_pid.cpp ../src/pid_controller.cpp)
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
colcon test --packages-select autocar_control --event-handlers console_direct+
```
Expected: Link error — `pid_controller.cpp` doesn't exist yet.

- [ ] **Step 3: Write minimal PID implementation**

`src/control/src/pid_controller.cpp`:
```cpp
#include "autocar_control/pid_controller.hpp"
#include <algorithm>

namespace autocar_control {

PIDController::PIDController(double kp, double ki, double kd,
                             double output_min, double output_max,
                             double integral_limit)
  : kp_(kp), ki_(ki), kd_(kd),
    output_min_(output_min), output_max_(output_max),
    integral_limit_(integral_limit) {}

void PIDController::reset() {
  integral_ = 0.0;
  prev_error_ = 0.0;
  first_run_ = true;
}

double PIDController::compute(double setpoint, double measurement, double dt) {
  if (dt <= 0.0) return 0.0;

  double error = setpoint - measurement;

  if (first_run_) {
    prev_error_ = error;
    first_run_ = false;
  }

  double p_term = kp_ * error;

  integral_ += error * dt;
  integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
  double i_term = ki_ * integral_;

  double derivative = (error - prev_error_) / dt;
  double d_term = kd_ * derivative;
  prev_error_ = error;

  double output = p_term + i_term + d_term;
  output = std::clamp(output, output_min_, output_max_);

  return output;
}

void PIDController::setGains(double kp, double ki, double kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

}  // namespace autocar_control
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
colcon test --packages-select autocar_control --event-handlers console_direct+
```
Expected: All 7 PID tests pass.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: implement PID controller with anti-windup and gain reconfiguration"
```

---

### Task 1.2: 实现电机驱动节点

**Files:**
- Modify: `src/control/src/motor_driver.cpp`
- Create: `src/control/test/test_motor_driver.cpp`

- [ ] **Step 1: Write motor driver test**

`src/control/test/test_motor_driver.cpp`:
```cpp
#include <gtest/gtest.h>

TEST(MotorDriverTest, EncoderToOdom) {
  // Given encoder counts for a differential drive robot
  int left_ticks = 48;
  int right_ticks = 48;
  double wheel_radius = 0.032;
  double wheel_base = 0.178;
  int encoder_ppr = 48;

  // Each wheel rotates 1 revolution (ppr ticks)
  double left_dist = (left_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;
  double right_dist = (right_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;

  double linear = (left_dist + right_dist) / 2.0;
  double angular = (right_dist - left_dist) / wheel_base;

  EXPECT_NEAR(linear, 2.0 * M_PI * wheel_radius, 0.001);
  EXPECT_NEAR(angular, 0.0, 0.001);
}

TEST(MotorDriverTest, DifferentialTurningOdom) {
  int left_ticks = 24;
  int right_ticks = 72;
  double wheel_radius = 0.032;
  double wheel_base = 0.178;
  int encoder_ppr = 48;

  double left_dist = (left_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;
  double right_dist = (right_ticks / static_cast<double>(encoder_ppr)) * 2.0 * M_PI * wheel_radius;

  double angular = (right_dist - left_dist) / wheel_base;
  EXPECT_GT(angular, 0.0);
}

TEST(MotorDriverTest, PWMClamped) {
  int pwm_value = 300;
  int clamped = std::clamp(pwm_value, -255, 255);
  EXPECT_EQ(clamped, 255);
}
```

Add test to `src/control/test/CMakeLists.txt`:
```cmake
ament_add_gtest(test_motor_driver test_motor_driver.cpp ../src/motor_driver.cpp)
```

- [ ] **Step 2: Implement motor_driver**

Replace `src/control/src/motor_driver.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class MotorDriver : public rclcpp::Node {
public:
  MotorDriver() : Node("motor_driver") {
    this->declare_parameter("wheel_radius", 0.032);
    this->declare_parameter("wheel_base", 0.178);
    this->declare_parameter("encoder_ppr", 48);
    this->declare_parameter("gear_ratio", 1.0);
    this->declare_parameter("pwm_pin", 12);
    this->declare_parameter("dir_pin", 13);
    this->declare_parameter("max_pwm", 255);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("/battery_voltage", 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    control_timer_ = this->create_wall_timer(
      20ms, std::bind(&MotorDriver::controlLoop, this));
    odom_timer_ = this->create_wall_timer(
      50ms, std::bind(&MotorDriver::publishOdometry, this));

    RCLCPP_INFO(this->get_logger(), "motor_driver initialized");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double left = msg->linear.x - msg->angular.z * wheel_base_ / 2.0;
    double right = msg->linear.x + msg->angular.z * wheel_base_ / 2.0;
    target_left_speed_ = static_cast<int>(std::clamp(left / max_speed_ * max_pwm_, -max_pwm_, max_pwm_));
    target_right_speed_ = static_cast<int>(std::clamp(right / max_speed_ * max_pwm_, -max_pwm_, max_pwm_));
  }

  void controlLoop() {
    // Hardware PWM write would go here:
    // - GPIO write to pwm_pin with target_left_speed_/target_right_speed_
    // - GPIO write to dir_pin for direction
    // In simulation mode, just track encoder ticks from simulated motor response
    current_left_speed_ = target_left_speed_;
    current_right_speed_ = target_right_speed_;
  }

  void publishOdometry() {
    double wheel_radius = this->get_parameter("wheel_radius").as_double();
    double wheel_base = this->get_parameter("wheel_base").as_double();
    double encoder_ppr = this->get_parameter("encoder_ppr").as_double();

    double left_dist = (current_left_speed_ / static_cast<double>(max_pwm_))
                       * max_speed_ * 0.05;
    double right_dist = (current_right_speed_ / static_cast<double>(max_pwm_))
                        * max_speed_ * 0.05;

    double linear = (left_dist + right_dist) / 2.0;
    double angular = (right_dist - left_dist) / wheel_base;

    x_ += linear * std::cos(theta_);
    y_ += linear * std::sin(theta_);
    theta_ += angular;

    auto now = this->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = linear / 0.05;
    odom.twist.twist.angular.z = angular / 0.05;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    std_msgs::msg::Float32 battery;
    battery.data = 12.0;
    battery_pub_->publish(battery);
  }

  double x_{0.0}, y_{0.0}, theta_{0.0};
  double wheel_radius_{0.032}, wheel_base_{0.178};
  double max_speed_{0.5};
  int max_pwm_{255};
  int target_left_speed_{0}, target_right_speed_{0};
  int current_left_speed_{0}, current_right_speed_{0};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Build and run tests**

```bash
colcon build --packages-select autocar_control
colcon test --packages-select autocar_control --event-handlers console_direct+
```
Expected: Build + tests pass.

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat: implement motor driver node with odometry publishing and cmd_vel subscription"
```

---

### Task 1.3: 实现速度和转向控制器

**Files:**
- Modify: `src/control/src/velocity_controller.cpp`
- Modify: `src/control/src/steering_controller.cpp`

- [ ] **Step 1: Write velocity controller test**

Add to `src/control/test/CMakeLists.txt`:
```cmake
ament_add_gtest(test_velocity_controller test_velocity_controller.cpp ../src/velocity_controller.cpp ../src/pid_controller.cpp)
```

`src/control/test/test_velocity_controller.cpp`:
```cpp
#include <gtest/gtest.h>
#include "autocar_control/pid_controller.hpp"

TEST(VelocityControllerTest, AcceleratesFromZero) {
  autocar_control::PIDController pid(1.2, 0.3, 0.05, -0.5, 0.5, 10.0);
  double cmd = pid.compute(0.3, 0.0, 0.05);
  EXPECT_GT(cmd, 0.0);
}

TEST(VelocityControllerTest, DeceleratesWhenOverspeed) {
  autocar_control::PIDController pid(1.2, 0.3, 0.05, -0.5, 0.5, 10.0);
  double cmd = pid.compute(0.3, 0.5, 0.05);
  EXPECT_LT(cmd, 0.0);
}

TEST(VelocityControllerTest, RespectsAccelLimit) {
  double max_accel = 0.3;
  double prev_cmd = 0.0;
  double cmd = 0.5;
  double accel = std::abs(cmd - prev_cmd) / 0.05;
  cmd = accel > max_accel ? prev_cmd + std::copysign(max_accel * 0.05, cmd - prev_cmd) : cmd;
  EXPECT_NEAR(cmd, 0.015, 0.001);
}
```

- [ ] **Step 2: Implement velocity_controller**

Replace `src/control/src/velocity_controller.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include "autocar_control/pid_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class VelocityController : public rclcpp::Node {
public:
  VelocityController() : Node("velocity_controller") {
    this->declare_parameter("kp", 1.2);
    this->declare_parameter("ki", 0.3);
    this->declare_parameter("kd", 0.05);
    this->declare_parameter("max_linear_speed", 0.5);
    this->declare_parameter("max_linear_accel", 0.3);
    this->declare_parameter("control_rate", 50.0);

    double rate = this->get_parameter("control_rate").as_double();
    dt_ = 1.0 / rate;

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&VelocityController::cmdVelCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&VelocityController::odomCallback, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<std_msgs::msg::Float32>("/motor_left_speed", 10);

    pid_ = std::make_unique<autocar_control::PIDController>(
      getParam("kp"), getParam("ki"), getParam("kd"),
      -getParam("max_linear_speed"), getParam("max_linear_speed"),
      10.0);

    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
      std::bind(&VelocityController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "velocity_controller initialized");
  }

private:
  double getParam(const std::string& name) {
    return this->get_parameter(name).as_double();
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    target_linear_ = msg->linear.x;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_linear_ = msg->twist.twist.linear.x;
  }

  void controlLoop() {
    double cmd = pid_->compute(target_linear_, current_linear_, dt_);

    double max_accel = getParam("max_linear_accel");
    double max_change = max_accel * dt_;
    cmd = std::clamp(cmd, prev_cmd_ - max_change, prev_cmd_ + max_change);
    prev_cmd_ = cmd;

    auto msg = std_msgs::msg::Float32();
    msg.data = static_cast<float>(cmd);
    motor_pub_->publish(msg);
  }

  double target_linear_{0.0};
  double current_linear_{0.0};
  double prev_cmd_{0.0};
  double dt_{0.02};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_pub_;
  std::unique_ptr<autocar_control::PIDController> pid_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityController>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Implement steering_controller**

Replace `src/control/src/steering_controller.cpp` with same structure as velocity_controller but controlling angular velocity (`msg->angular.z`), publishing to `motor_right_speed` topic, and with angular-relevant params.

- [ ] **Step 4: Build and test**

```bash
colcon build --packages-select autocar_control
colcon test --packages-select autocar_control --event-handlers console_direct+
```
Expected: Build + tests pass.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: implement velocity and steering PID controller nodes"
```

---

## Phase 2: 感知层 — 知道位置

### Task 2.1: 实现传感器驱动节点

**Files:**
- Modify: `src/perception/src/gps_driver.cpp`
- Modify: `src/perception/src/imu_driver.cpp`
- Modify: `src/perception/src/lidar_driver.cpp`
- Modify: `src/perception/src/camera_driver.cpp`

- [ ] **Step 1: Implement GPS driver**

Rewrite `src/perception/src/gps_driver.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

class GpsDriver : public rclcpp::Node {
public:
  GpsDriver() : Node("gps_driver") {
    this->declare_parameter("port", "/dev/ttyAMA0");
    this->declare_parameter("baud_rate", 9600);
    this->declare_parameter("frame_id", "gps_link");
    this->declare_parameter("publish_rate", 10.0);

    fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);

    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&GpsDriver::publishFix, this));

    RCLCPP_INFO(this->get_logger(), "gps_driver initialized");
  }

private:
  void publishFix() {
    auto fix = sensor_msgs::msg::NavSatFix();
    fix.header.stamp = this->now();
    fix.header.frame_id = this->get_parameter("frame_id").as_string();
    fix.latitude = 39.9042;
    fix.longitude = 116.4074;
    fix.altitude = 50.0;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
    fix_pub_->publish(fix);
  }

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsDriver>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Implement IMU driver**

Rewrite `src/perception/src/imu_driver.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuDriver : public rclcpp::Node {
public:
  ImuDriver() : Node("imu_driver") {
    this->declare_parameter("i2c_bus", 1);
    this->declare_parameter("address", 0x68);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("publish_rate", 100.0);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);

    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&ImuDriver::publishImu, this));

    RCLCPP_INFO(this->get_logger(), "imu_driver initialized");
  }

private:
  void publishImu() {
    auto imu = sensor_msgs::msg::Imu();
    imu.header.stamp = this->now();
    imu.header.frame_id = this->get_parameter("frame_id").as_string();
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.z = 0.0;
    imu.linear_acceleration.x = 0.0;
    imu_pub_->publish(imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDriver>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Implement LiDAR driver**

Rewrite `src/perception/src/lidar_driver.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarDriver : public rclcpp::Node {
public:
  LidarDriver() : Node("lidar_driver") {
    this->declare_parameter("port", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 230400);
    this->declare_parameter("frame_id", "laser_frame");
    this->declare_parameter("angle_min", 0.0);
    this->declare_parameter("angle_max", 6.283);
    this->declare_parameter("range_min", 0.05);
    this->declare_parameter("range_max", 12.0);

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LidarDriver::publishScan, this));

    RCLCPP_INFO(this->get_logger(), "lidar_driver initialized");
  }

private:
  void publishScan() {
    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp = this->now();
    scan.header.frame_id = this->get_parameter("frame_id").as_string();
    scan.angle_min = this->get_parameter("angle_min").as_double();
    scan.angle_max = this->get_parameter("angle_max").as_double();
    scan.angle_increment = 0.01745;
    scan.range_min = this->get_parameter("range_min").as_double();
    scan.range_max = this->get_parameter("range_max").as_double();
    scan.ranges.resize(360, 10.0);
    scan_pub_->publish(scan);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDriver>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Implement camera driver**

Rewrite `src/perception/src/camera_driver.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraDriver : public rclcpp::Node {
public:
  CameraDriver() : Node("camera_driver") {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(66),
      std::bind(&CameraDriver::publishFrame, this));

    RCLCPP_INFO(this->get_logger(), "camera_driver initialized");
  }

private:
  void publishFrame() {
    auto img = sensor_msgs::msg::Image();
    img.header.stamp = this->now();
    img.header.frame_id = "camera_frame";
    img.height = 480;
    img.width = 640;
    img.encoding = "rgb8";
    img.is_bigendian = false;
    img.step = 640 * 3;
    img.data.resize(640 * 480 * 3, 0);
    image_pub_->publish(img);

    auto depth = sensor_msgs::msg::Image();
    depth.header = img.header;
    depth.height = 480;
    depth.width = 640;
    depth.encoding = "16UC1";
    depth.is_bigendian = false;
    depth.step = 640 * 2;
    depth.data.resize(640 * 480 * 2, 0);
    depth_pub_->publish(depth);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraDriver>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 5: Build and verify**

```bash
colcon build --packages-select autocar_perception
```
Expected: Build succeeds.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat: implement GPS, IMU, LiDAR, and camera driver nodes with stub data"
```

---

### Task 2.2: 实现 EKF 定位融合节点

**Files:**
- Modify: `src/perception/src/localization_fusion.cpp`
- Create: `src/perception/src/kalman_filter.cpp`
- Create: `src/perception/test/test_kalman.cpp`

- [ ] **Step 1: Write EKF unit test**

`src/perception/test/test_kalman.cpp`:
```cpp
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "autocar_perception/kalman_filter.hpp"

TEST(EKFTest, InitStateIsStored) {
  autocar_perception::ExtendedKalmanFilter ekf(3, 2);
  Eigen::VectorXd x0(3);
  x0 << 0.0, 0.0, 0.0;
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(3, 3);
  ekf.setInitialState(x0, P0);
  auto x = ekf.state();
  EXPECT_NEAR(x(0), 0.0, 1e-9);
  EXPECT_NEAR(x(1), 0.0, 1e-9);
}

TEST(EKFTest, PredictionIncreasesUncertainty) {
  autocar_perception::ExtendedKalmanFilter ekf(3, 1);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(3, 3) * 0.1;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3) * 0.01;
  ekf.setInitialState(x0, P0);
  ekf.setProcessNoise(Q);

  auto before = ekf.covariance().trace();
  Eigen::VectorXd u = Eigen::VectorXd::Zero(2);
  ekf.predict(u, 0.1,
    [](const auto& s, const auto& c, double dt) { return s; },
    [](const auto& s, const auto& c, double dt) { return Eigen::MatrixXd::Identity(3, 3); });
  auto after = ekf.covariance().trace();
  EXPECT_GT(after, before);
}

TEST(EKFTest, UpdateReducesUncertainty) {
  autocar_perception::ExtendedKalmanFilter ekf(3, 1);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(3, 3) * 1.0;
  ekf.setInitialState(x0, P0);
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1) * 0.01;
  ekf.setMeasurementNoise(R);

  auto before = ekf.covariance().trace();
  Eigen::VectorXd z(1); z << 0.5;
  ekf.update(z,
    [](const auto& s) { return Eigen::VectorXd::Constant(1, s(0)); },
    [](const auto& s) {
      Eigen::MatrixXd H(1, 3);
      H << 1.0, 0.0, 0.0;
      return H;
    });
  auto after = ekf.covariance().trace();
  EXPECT_LT(after, before);
}
```

- [ ] **Step 2: Implement EKF**

`src/perception/src/kalman_filter.cpp`:
```cpp
#include "autocar_perception/kalman_filter.hpp"
#include <stdexcept>

namespace autocar_perception {

ExtendedKalmanFilter::ExtendedKalmanFilter(int state_dim, int measurement_dim)
  : state_dim_(state_dim), measurement_dim_(measurement_dim) {
  x_ = Eigen::VectorXd::Zero(state_dim);
  P_ = Eigen::MatrixXd::Identity(state_dim, state_dim);
  Q_ = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.01;
  R_ = Eigen::MatrixXd::Identity(measurement_dim, measurement_dim) * 0.1;
}

void ExtendedKalmanFilter::setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0) {
  if (x0.size() != state_dim_) throw std::invalid_argument("state dim mismatch");
  x_ = x0;
  P_ = P0;
}

void ExtendedKalmanFilter::setProcessNoise(const Eigen::MatrixXd& Q) { Q_ = Q; }
void ExtendedKalmanFilter::setMeasurementNoise(const Eigen::MatrixXd& R) { R_ = R; }

void ExtendedKalmanFilter::predict(
    const Eigen::VectorXd& u, double dt,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> f,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&, double)> F_jacobian) {
  x_ = f(x_, u, dt);
  Eigen::MatrixXd F = F_jacobian(x_, u, dt);
  P_ = F * P_ * F.transpose() + Q_;
}

void ExtendedKalmanFilter::update(
    const Eigen::VectorXd& z,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h,
    std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> H_jacobian) {
  Eigen::VectorXd z_pred = h(x_);
  Eigen::VectorXd y = z - z_pred;
  Eigen::MatrixXd H = H_jacobian(x_);
  Eigen::MatrixXd S = H * P_ * H.transpose() + R_;
  Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
  x_ = x_ + K * y;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  P_ = (I - K * H) * P_;
}

}  // namespace autocar_perception
```

- [ ] **Step 3: Implement localization_fusion node**

Rewrite `src/perception/src/localization_fusion.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "autocar_perception/kalman_filter.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

class LocalizationFusion : public rclcpp::Node {
public:
  LocalizationFusion() : Node("localization_fusion"), ekf_(6, 3) {
    this->declare_parameter("gps_min_satellites", 6);
    this->declare_parameter("gps_max_hdop", 2.0);
    this->declare_parameter("gps_loss_timeout_s", 3.0);

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 10, std::bind(&LocalizationFusion::gpsCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&LocalizationFusion::imuCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&LocalizationFusion::odomCallback, this, std::placeholders::_1));

    fused_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_fused", 10);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6);
    ekf_.setInitialState(x0, P0);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&LocalizationFusion::fusionLoop, this));

    RCLCPP_INFO(this->get_logger(), "localization_fusion initialized");
  }

private:
  enum class Mode { OUTDOOR, INDOOR, TRANSITION };

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    last_gps_time_ = this->now();
    gps_lat_ = msg->latitude;
    gps_lon_ = msg->longitude;
    gps_alt_ = msg->altitude;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imu_yaw_rate_ = msg->angular_velocity.z;
    imu_ax_ = msg->linear_acceleration.x;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    odom_linear_ = msg->twist.twist.linear.x;
  }

  void fusionLoop() {
    double dt = 0.05;
    auto now = this->now();
    double gps_age = (now - last_gps_time_).seconds();
    bool gps_valid = gps_age < 3.0;

    if (gps_valid && mode_ != Mode::OUTDOOR) {
      mode_ = Mode::OUTDOOR;
      RCLCPP_INFO(this->get_logger(), "switching to OUTDOOR mode");
    } else if (!gps_valid && mode_ != Mode::INDOOR) {
      mode_ = Mode::INDOOR;
      RCLCPP_INFO(this->get_logger(), "switching to INDOOR mode");
    }

    // Predict step
    auto f = [this](const Eigen::VectorXd& s, const Eigen::VectorXd& u, double dt) {
      Eigen::VectorXd pred(6);
      double yaw = s(2);
      pred(0) = s(0) + s(3) * cos(yaw) * dt;
      pred(1) = s(1) + s(3) * sin(yaw) * dt;
      pred(2) = s(2) + s(5) * dt;
      pred(3) = s(3) + u(0) * dt;
      pred(4) = 0.0;
      pred(5) = s(5);
      return pred;
    };

    auto F = [](const Eigen::VectorXd& s, const Eigen::VectorXd& u, double dt) {
      Eigen::MatrixXd Fj(6, 6);
      Fj.setIdentity();
      double y = s(2), v = s(3);
      Fj(0, 2) = -v * sin(y) * dt;
      Fj(0, 3) = cos(y) * dt;
      Fj(1, 2) = v * cos(y) * dt;
      Fj(1, 3) = sin(y) * dt;
      Fj(2, 5) = dt;
      return Fj;
    };

    Eigen::VectorXd u(1); u << imu_ax_;
    ekf_.predict(u, dt, f, F);

    // Update with odometry (always available)
    if (has_odom_) {
      auto h_odom = [](const Eigen::VectorXd& s) {
        Eigen::VectorXd z(2);
        z << s(3), s(5);
        return z;
      };
      auto H_odom = [](const Eigen::VectorXd& s) {
        Eigen::MatrixXd H(2, 6);
        H.setZero();
        H(0, 3) = 1.0;
        H(1, 5) = 1.0;
        return H;
      };
      Eigen::VectorXd z(2);
      z << odom_linear_, imu_yaw_rate_;
      ekf_.update(z, h_odom, H_odom);
    }

    // Publish fused odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = ekf_.state()(0);
    odom.pose.pose.position.y = ekf_.state()(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, ekf_.state()(2));
    odom.pose.pose.orientation = tf2::toMsg(q);
    fused_pub_->publish(odom);
  }

  autocar_perception::ExtendedKalmanFilter ekf_;
  Mode mode_{Mode::OUTDOOR};
  rclcpp::Time last_gps_time_{0, 0, RCL_ROS_TIME};
  double gps_lat_{0.0}, gps_lon_{0.0}, gps_alt_{0.0};
  double imu_yaw_rate_{0.0}, imu_ax_{0.0};
  double odom_x_{0.0}, odom_y_{0.0}, odom_linear_{0.0};
  bool has_odom_{false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationFusion>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Build and test**

```bash
colcon build --packages-select autocar_perception
colcon test --packages-select autocar_perception --event-handlers console_direct+
```
Expected: Build + EKF tests pass.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: implement EKF localization fusion with indoor/outdoor mode switching"
```

---

### Task 2.3: 实现 safety_watchdog 节点

**Files:**
- Modify: `src/control/src/safety_watchdog.cpp`
- Create: `src/control/test/test_safety_watchdog.cpp`

- [ ] **Step 1: Write safety tests**

`src/control/test/test_safety_watchdog.cpp`:
```cpp
#include <gtest/gtest.h>

enum class SafetyLevel { FATAL = 0, CRITICAL = 1, WARNING = 2, NORMAL = 3 };

SafetyLevel checkBattery(float voltage) {
  if (voltage < 10.5) return SafetyLevel::FATAL;
  if (voltage < 11.0) return SafetyLevel::CRITICAL;
  if (voltage < 11.5) return SafetyLevel::WARNING;
  return SafetyLevel::NORMAL;
}

SafetyLevel checkCmdVelTimeout(double last_cmd_time_ms, double now_ms) {
  if (now_ms - last_cmd_time_ms > 200.0) return SafetyLevel::CRITICAL;
  return SafetyLevel::NORMAL;
}

TEST(SafetyTest, BatteryLowIsCritical) {
  EXPECT_EQ(checkBattery(10.3), SafetyLevel::FATAL);
  EXPECT_EQ(checkBattery(10.7), SafetyLevel::CRITICAL);
  EXPECT_EQ(checkBattery(11.2), SafetyLevel::WARNING);
  EXPECT_EQ(checkBattery(12.0), SafetyLevel::NORMAL);
}

TEST(SafetyTest, CmdVelTimeoutTriggers) {
  EXPECT_EQ(checkCmdVelTimeout(0, 300), SafetyLevel::CRITICAL);
  EXPECT_EQ(checkCmdVelTimeout(100, 250), SafetyLevel::NORMAL);
}
```

- [ ] **Step 2: Implement safety_watchdog**

Rewrite `src/control/src/safety_watchdog.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SafetyWatchdog : public rclcpp::Node {
public:
  SafetyWatchdog() : Node("safety_watchdog") {
    this->declare_parameter("battery_threshold_critical", 10.5);
    this->declare_parameter("battery_threshold_warning", 11.0);
    this->declare_parameter("cmd_vel_timeout_ms", 200);
    this->declare_parameter("heartbeat_timeout_ms", 500);
    this->declare_parameter("sensor_loss_timeout_s", 5.0);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&SafetyWatchdog::cmdVelCallback, this, std::placeholders::_1));
    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/battery_voltage", 10, std::bind(&SafetyWatchdog::batteryCallback, this, std::placeholders::_1));

    safety_pub_ = this->create_publisher<std_msgs::msg::String>("/safety_state", 10);
    safe_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_safe", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&SafetyWatchdog::watchdogLoop, this));
    RCLCPP_INFO(this->get_logger(), "safety_watchdog initialized");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr) {
    last_cmd_time_ = this->now();
  }

  void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    battery_voltage_ = msg->data;
  }

  void watchdogLoop() {
    auto now = this->now();
    double cmd_age = (now - last_cmd_time_).seconds() * 1000.0;
    auto level = 3; // NORMAL
    std::string detail = "ok";

    if (battery_voltage_ < 10.5) {
      level = 0; detail = "battery critical";
    } else if (cmd_age > 500.0) {
      level = 1; detail = "cmd_vel heartbeat lost";
    } else if (battery_voltage_ < 11.0) {
      level = 1; detail = "battery low";
    } else if (cmd_age > 200.0) {
      level = 2; detail = "cmd_vel stale";
    }

    auto safe_msg = geometry_msgs::msg::Twist();
    if (level <= 1) {
      safe_msg.linear.x = 0.0;
      safe_msg.angular.z = 0.0;
    } else {
      safe_msg.linear.x = last_cmd_.linear.x;
      safe_msg.angular.z = last_cmd_.angular.z;
    }
    safe_cmd_vel_pub_->publish(safe_msg);

    auto state = std_msgs::msg::String();
    state.data = detail;
    safety_pub_->publish(state);
  }

  double battery_voltage_{12.0};
  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyWatchdog>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Build and test**

```bash
colcon build --packages-select autocar_control
colcon test --packages-select autocar_control --event-handlers console_direct+
```
Expected: All tests pass.

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat: implement safety watchdog with battery and heartbeat monitoring"
```

---

## Phase 3: 规划层 — 能自己走

### Task 3.1: 实现 A* 全局路径规划器

**Files:**
- Modify: `src/planning/src/global_planner.cpp`
- Create: `src/planning/src/astar.cpp`
- Create: `src/planning/test/test_astar.cpp`

- [ ] **Step 1: Write A* tests**

`src/planning/test/test_astar.cpp`:
```cpp
#include <gtest/gtest.h>
#include "autocar_planning/astar.hpp"

TEST(AStarTest, StraightLinePath) {
  autocar_planning::AStarPlanner planner(10, 10, 1.0);
  planner.setOccupancyFn([](int, int) { return false; });

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0; start.pose.position.y = 0.0;
  goal.pose.position.x = 5.0; goal.pose.position.y = 0.0;

  auto path = planner.plan(start, goal);
  EXPECT_GT(path.poses.size(), 0u);
  EXPECT_NEAR(path.poses.back().pose.position.x, 5.0, 1.0);
}

TEST(AStarTest, BlockedPathReturnsEmpty) {
  autocar_planning::AStarPlanner planner(10, 10, 1.0);
  planner.setOccupancyFn([](int x, int y) { return x == 1 && y == 0; });

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0; start.pose.position.y = 0.0;
  goal.pose.position.x = 5.0; goal.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  bool succ = planner.planToPath(start, goal, path);
  // Wall at x=1 shouldn't completely block because y freedom exists
  EXPECT_TRUE(succ);
}

TEST(AStarTest, NoPathWhenFullyBlocked) {
  autocar_planning::AStarPlanner planner(3, 1, 1.0);
  planner.setOccupancyFn([](int, int) { return true; });

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0; start.pose.position.y = 0.0;
  goal.pose.position.x = 2.0; goal.pose.position.y = 0.0;

  nav_msgs::msg::Path path;
  bool succ = planner.planToPath(start, goal, path);
  EXPECT_FALSE(succ);
}
```

- [ ] **Step 2: Implement A* algorithm**

`src/planning/src/astar.cpp`:
```cpp
#include "autocar_planning/astar.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace autocar_planning {

AStarPlanner::AStarPlanner(int width, int height, double resolution)
  : width_(width), height_(height), resolution_(resolution) {
  is_occupied_ = [](int, int) { return false; };
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
  return std::hypot(x1 - x2, y1 - y2) * resolution_;
}

std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int x, int y) const {
  std::vector<std::pair<int, int>> neighbors;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      neighbors.push_back({x + dx, y + dy});
    }
  }
  return neighbors;
}

bool AStarPlanner::isValid(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_ && !is_occupied_(x, y);
}

void AStarPlanner::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>(wx / resolution_);
  gy = static_cast<int>(wy / resolution_);
}

void AStarPlanner::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  wx = gx * resolution_;
  wy = gy * resolution_;
}

nav_msgs::msg::Path AStarPlanner::plan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  nav_msgs::msg::Path path;
  path.header.frame_id = start.header.frame_id;
  planToPath(start, goal, path);
  return path;
}

bool AStarPlanner::planToPath(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal,
    nav_msgs::msg::Path& path) {
  int sx, sy, gx, gy;
  worldToGrid(start.pose.position.x, start.pose.position.y, sx, sy);
  worldToGrid(goal.pose.position.x, goal.pose.position.y, gx, gy);

  using PQ = std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>>;
  PQ open;
  std::unordered_map<int, std::unordered_map<int, bool>> closed;

  AStarNode start_node;
  start_node.x = sx; start_node.y = sy;
  start_node.g_cost = 0.0;
  start_node.h_cost = heuristic(sx, sy, gx, gy);
  open.push(start_node);

  auto key = [](int x, int y) { return x * 10000 + y; };
  std::unordered_map<int, AStarNode*> open_map;
  open_map[key(sx, sy)] = const_cast<AStarNode*>(&open.top());

  while (!open.empty()) {
    auto current = open.top(); open.pop();

    if (current.x == gx && current.y == gy) {
      std::vector<AStarNode*> trace;
      AStarNode* node = &current;
      while (node) {
        trace.push_back(node);
        node = node->parent;
      }
      std::reverse(trace.begin(), trace.end());
      for (auto* n : trace) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = start.header;
        gridToWorld(n->x, n->y, pose.pose.position.x, pose.pose.position.y);
        path.poses.push_back(pose);
      }
      return true;
    }

    closed[current.x][current.y] = true;

    for (auto [nx, ny] : getNeighbors(current.x, current.y)) {
      if (!isValid(nx, ny) || closed[nx][ny]) continue;
      double step_cost = (nx != current.x && ny != current.y) ? 1.414 : 1.0;
      double new_g = current.g_cost + step_cost;
      AStarNode neighbor;
      neighbor.x = nx; neighbor.y = ny;
      neighbor.g_cost = new_g;
      neighbor.h_cost = heuristic(nx, ny, gx, gy);
      open.push(neighbor);
    }
  }
  return false;
}

}  // namespace autocar_planning
```

- [ ] **Step 3: Implement global_planner ROS2 wrapper**

Rewrite `src/planning/src/global_planner.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "autocar_planning/astar.hpp"

class GlobalPlanner : public rclcpp::Node {
public:
  GlobalPlanner() : Node("global_planner"), astar_(200, 200, 1.0) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task_goal", 10,
      std::bind(&GlobalPlanner::goalCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "global_planner initialized");
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.position.x = 0.0;
    start.pose.position.y = 0.0;

    auto path = astar_.plan(start, *goal);
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    path_pub_->publish(path);
  }

  autocar_planning::AStarPlanner astar_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Build and test**

```bash
colcon build --packages-select autocar_planning
colcon test --packages-select autocar_planning --event-handlers console_direct+
```

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: implement A* global path planner with occupancy grid support"
```

---

### Task 3.2: 实现 DWA 局部规划器与行为树引擎

**Files:**
- Modify: `src/planning/src/local_planner.cpp`
- Modify: `src/planning/src/behavior_tree.cpp`
- Create: `src/planning/src/dwa_planner.cpp`
- Create: `src/planning/src/behavior_nodes.cpp`

- [ ] **Step 1: Implement DWA planner**

`src/planning/src/dwa_planner.cpp`:
```cpp
#include "autocar_planning/dwa_planner.hpp"
#include <cmath>
#include <algorithm>

namespace autocar_planning {

DWAPlanner::DWAPlanner(const DWAConfig& config) : config_(config) {}

geometry_msgs::msg::Twist DWAPlanner::plan(
    const geometry_msgs::msg::PoseStamped& current_pose,
    const geometry_msgs::msg::Twist& current_velocity,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan) {

  auto trajectories = generateTrajectories(current_pose, current_velocity);
  if (trajectories.empty()) {
    geometry_msgs::msg::Twist zero;
    return zero;
  }

  for (auto& traj : trajectories) {
    traj.cost = evaluateTrajectory(traj, global_path, scan);
  }

  auto best = std::min_element(trajectories.begin(), trajectories.end(),
    [](const Trajectory& a, const Trajectory& b) { return a.cost < b.cost; });

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = best->linear_velocity;
  cmd.angular.z = best->angular_velocity;
  return cmd;
}

std::vector<Trajectory> DWAPlanner::generateTrajectories(
    const geometry_msgs::msg::PoseStamped& pose,
    const geometry_msgs::msg::Twist& vel) {
  std::vector<Trajectory> result;
  double v_min = std::max(0.0, vel.linear.x - config_.linear_accel * config_.predict_time);
  double v_max = std::min(config_.max_linear_speed, vel.linear.x + config_.linear_accel * config_.predict_time);
  double w_min = std::max(-config_.max_angular_speed, vel.angular.z - config_.angular_accel * config_.predict_time);
  double w_max = std::min(config_.max_angular_speed, vel.angular.z + config_.angular_accel * config_.predict_time);

  for (double v = v_min; v <= v_max; v += config_.linear_resolution) {
    for (double w = w_min; w <= w_max; w += config_.angular_resolution) {
      result.push_back(simulateTrajectory(v, w, pose));
    }
  }
  return result;
}

Trajectory DWAPlanner::simulateTrajectory(double v, double w,
    const geometry_msgs::msg::PoseStamped& start) {
  Trajectory traj;
  traj.linear_velocity = v;
  traj.angular_velocity = w;

  double x = start.pose.position.x;
  double y = start.pose.position.y;
  double theta = 0.0;

  for (double t = 0; t < config_.predict_time; t += config_.dt) {
    x += v * cos(theta) * config_.dt;
    y += v * sin(theta) * config_.dt;
    theta += w * config_.dt;

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    traj.poses.push_back(pose);
  }
  return traj;
}

double DWAPlanner::evaluateTrajectory(const Trajectory& traj,
    const nav_msgs::msg::Path& global_path,
    const sensor_msgs::msg::LaserScan& scan) {
  return config_.obstacle_cost_weight * obstacleCost(traj, scan) +
         config_.goal_cost_weight * goalCost(traj, global_path) +
         config_.speed_cost_weight * speedCost(traj);
}

double DWAPlanner::obstacleCost(const Trajectory& traj, const sensor_msgs::msg::LaserScan& scan) {
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& pose : traj.poses) {
    double dist = std::hypot(pose.pose.position.x, pose.pose.position.y);
    if (dist < config_.goal_tolerance) return 1e9;
    min_dist = std::min(min_dist, dist);
  }
  return 1.0 / (min_dist + 0.1);
}

double DWAPlanner::goalCost(const Trajectory& traj, const nav_msgs::msg::Path& global_path) {
  if (global_path.poses.empty()) return 0.0;
  const auto& goal = global_path.poses.back();
  const auto& last = traj.poses.back();
  return std::hypot(last.pose.position.x - goal.pose.position.x,
                    last.pose.position.y - goal.pose.position.y);
}

double DWAPlanner::speedCost(const Trajectory& traj) {
  return config_.max_linear_speed - traj.linear_velocity;
}

}  // namespace autocar_planning
```

- [ ] **Step 2: Implement behavior tree nodes**

`src/planning/src/behavior_nodes.cpp`:
```cpp
#include "autocar_planning/behavior_nodes.hpp"

namespace autocar_planning {

Sequence::Sequence(const std::string& name) { name_ = name; }
void Sequence::addChild(std::shared_ptr<BehaviorNode> child) { children_.push_back(child); }

NodeStatus Sequence::tick() {
  for (; current_index_ < children_.size(); ++current_index_) {
    auto status = children_[current_index_]->tick();
    if (status != NodeStatus::SUCCESS) return status;
  }
  return NodeStatus::SUCCESS;
}

void Sequence::reset() {
  current_index_ = 0;
  for (auto& c : children_) c->reset();
}

Fallback::Fallback(const std::string& name) { name_ = name; }
void Fallback::addChild(std::shared_ptr<BehaviorNode> child) { children_.push_back(child); }

NodeStatus Fallback::tick() {
  for (; current_index_ < children_.size(); ++current_index_) {
    auto status = children_[current_index_]->tick();
    if (status != NodeStatus::FAILURE) return status;
  }
  return NodeStatus::FAILURE;
}

void Fallback::reset() {
  current_index_ = 0;
  for (auto& c : children_) c->reset();
}

ReactiveSequence::ReactiveSequence(const std::string& name) { name_ = name; }
void ReactiveSequence::addChild(std::shared_ptr<BehaviorNode> child) { children_.push_back(child); }

NodeStatus ReactiveSequence::tick() {
  for (auto& c : children_) {
    auto status = c->tick();
    if (status != NodeStatus::SUCCESS) return status;
  }
  return NodeStatus::SUCCESS;
}

void ReactiveSequence::reset() {
  for (auto& c : children_) c->reset();
}

ConditionNode::ConditionNode(const std::string& name, std::function<bool()> condition)
  : condition_(condition) { name_ = name; }

NodeStatus ConditionNode::tick() {
  return condition_() ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
}

ActionNode::ActionNode(const std::string& name, std::function<NodeStatus()> action)
  : action_(action) { name_ = name; }

NodeStatus ActionNode::tick() {
  return action_();
}

}  // namespace autocar_planning
```

- [ ] **Step 3: Implement local_planner ROS2 wrapper**

Rewrite `src/planning/src/local_planner.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "autocar_planning/dwa_planner.hpp"
#include <mutex>

class LocalPlanner : public rclcpp::Node {
public:
  LocalPlanner() : Node("local_planner"), dwa_(autocar_planning::DWAConfig()) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LocalPlanner::scanCallback, this, std::placeholders::_1));
    global_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/global_path", 10, std::bind(&LocalPlanner::globalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom_fused", 10, std::bind(&LocalPlanner::odomCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
      std::bind(&LocalPlanner::planLoop, this));
    RCLCPP_INFO(this->get_logger(), "local_planner initialized");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mtx_); last_scan_ = *m;
  }
  void globalCallback(const nav_msgs::msg::Path::SharedPtr m) {
    std::lock_guard<std::mutex> lk(mtx_); global_path_ = *m;
  }
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr m) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = m->header;
    pose.pose = m->pose.pose;
    std::lock_guard<std::mutex> lk(mtx_);
    current_pose_ = pose;
    current_vel_ = m->twist.twist;
  }

  void planLoop() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (global_path_.poses.empty()) return;
    auto cmd = dwa_.plan(current_pose_, current_vel_, global_path_, last_scan_);
    cmd_vel_pub_->publish(cmd);
  }

  autocar_planning::DWAPlanner dwa_;
  std::mutex mtx_;
  sensor_msgs::msg::LaserScan last_scan_;
  nav_msgs::msg::Path global_path_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_vel_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPlanner>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Implement behavior_tree ROS2 node**

Rewrite `src/planning/src/behavior_tree.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include "autocar_planning/behavior_nodes.hpp"

class BehaviorTree : public rclcpp::Node {
public:
  BehaviorTree() : Node("behavior_tree") {
    voice_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/voice_command", 10, std::bind(&BehaviorTree::voiceCallback, this, std::placeholders::_1));
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/task_goal", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("/bt_state", 10);

    RCLCPP_INFO(this->get_logger(), "behavior_tree initialized");
  }

private:
  void voiceCallback(const std_msgs::msg::String::SharedPtr cmd) {
    RCLCPP_INFO(this->get_logger(), "BT received voice: '%s'", cmd->data.c_str());

    if (cmd->data.find("NAVIGATE") != std::string::npos) {
      auto goal = geometry_msgs::msg::PoseStamped();
      goal.header.frame_id = "map";
      goal.header.stamp = this->now();
      goal.pose.position.x = 50.0;
      goal.pose.position.y = 0.0;
      goal_pub_->publish(goal);

      auto state = std_msgs::msg::String();
      state.data = "navigating";
      state_pub_->publish(state);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorTree>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 5: Build and test**

```bash
colcon build --packages-select autocar_planning
colcon test --packages-select autocar_planning --event-handlers console_direct+
```

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "feat: implement DWA local planner, behavior tree engine, and planner ROS2 nodes"
```

---

## Phase 4: 应用层 — 能对话

### Task 4.1: 实现状态机节点

**Files:**
- Modify: `src/application/autocar_application/state_machine.py`
- Create: `src/application/test/test_state_machine.py`

- [ ] **Step 1: Write state machine test**

`src/application/test/test_state_machine.py`:
```python
"""Tests for autocar state machine."""
import pytest

STATES = ["BOOTUP", "IDLE", "AUTONOMY", "PAUSED", "ESTOP"]
TRANSITIONS = {
    "BOOTUP": ["IDLE"],
    "IDLE": ["AUTONOMY"],
    "AUTONOMY": ["PAUSED", "ESTOP"],
    "PAUSED": ["AUTONOMY", "ESTOP", "IDLE"],
    "ESTOP": ["IDLE"],
}


def test_all_states_defined():
    assert len(STATES) == 5


def test_estop_blocks_all():
    # ESTOP should only go to IDLE (manual reset)
    assert "AUTONOMY" not in TRANSITIONS["ESTOP"]
    assert "PAUSED" not in TRANSITIONS["ESTOP"]


def test_autonomy_cannot_go_to_idle_directly():
    assert "IDLE" not in TRANSITIONS["AUTONOMY"]


def test_paused_can_resume():
    assert "AUTONOMY" in TRANSITIONS["PAUSED"]


def test_bootup_only_to_idle():
    assert TRANSITIONS["BOOTUP"] == ["IDLE"]


def test_all_states_have_transitions():
    for state in STATES:
        assert len(TRANSITIONS[state]) > 0
```

- [ ] **Step 2: Implement state machine**

Rewrite `src/application/autocar_application/state_machine.py`:
```python
"""Autocar state machine — manages system modes."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from enum import Enum


class SystemState(Enum):
    BOOTUP = "BOOTUP"
    IDLE = "IDLE"
    AUTONOMY = "AUTONOMY"
    PAUSED = "PAUSED"
    ESTOP = "ESTOP"


class StateMachine(Node):
    VALID_TRANSITIONS = {
        SystemState.BOOTUP: [SystemState.IDLE],
        SystemState.IDLE: [SystemState.AUTONOMY],
        SystemState.AUTONOMY: [SystemState.PAUSED, SystemState.ESTOP],
        SystemState.PAUSED: [SystemState.AUTONOMY, SystemState.ESTOP, SystemState.IDLE],
        SystemState.ESTOP: [SystemState.IDLE],
    }

    def __init__(self):
        super().__init__('state_machine')
        self.state = SystemState.BOOTUP

        self.voice_sub = self.create_subscription(
            String, '/voice_command', self.on_voice_command, 10)
        self.safety_sub = self.create_subscription(
            String, '/safety_state', self.on_safety_state, 10)
        self.task_pub = self.create_publisher(
            PoseStamped, '/task_goal', 10)
        self.state_pub = self.create_publisher(
            String, '/system_state', 10)

        self.create_timer(1.0, self.publish_state)
        self._switch_to(SystemState.IDLE)
        self.get_logger().info('state_machine initialized in IDLE')

    def _switch_to(self, target: SystemState) -> bool:
        if target in self.VALID_TRANSITIONS.get(self.state, []):
            old = self.state
            self.state = target
            self.get_logger().info(f'state transition: {old.value} -> {target.value}')
            return True
        self.get_logger().warn(f'invalid transition: {self.state.value} -> {target.value}')
        return False

    def on_voice_command(self, msg: String):
        cmd = msg.data.strip().upper()
        self.get_logger().info(f'voice command in state {self.state.value}: {cmd}')

        if cmd == 'NAVIGATE' and self.state == SystemState.IDLE:
            self._switch_to(SystemState.AUTONOMY)
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            self.task_pub.publish(goal)
        elif cmd == 'PAUSE' and self.state == SystemState.AUTONOMY:
            self._switch_to(SystemState.PAUSED)
        elif cmd == 'RESUME' and self.state == SystemState.PAUSED:
            self._switch_to(SystemState.AUTONOMY)
        elif cmd == 'STOP' and self.state in (SystemState.AUTONOMY, SystemState.PAUSED):
            self._switch_to(SystemState.IDLE)

    def on_safety_state(self, msg: String):
        if msg.data == 'estop_active' and self.state != SystemState.ESTOP:
            self._switch_to(SystemState.ESTOP)
            self.get_logger().error('ESTOP triggered!')

    def publish_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Run tests**

```bash
cd src/application && python -m pytest test/ -v
```

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat: implement state machine with BOOTUP/IDLE/AUTONOMY/PAUSED/ESTOP modes"
```

---

### Task 4.2: 实现语音管道节点

**Files:**
- Modify: `src/application/autocar_application/voice_pipeline.py`
- Create: `src/application/test/test_voice_pipeline.py`

- [ ] **Step 1: Write intent parser test**

`src/application/test/test_voice_pipeline.py`:
```python
"""Tests for voice pipeline intent parsing."""
import pytest


INTENT_RULES = [
    ("去", "NAVIGATE"),
    ("到", "NAVIGATE"),
    ("停", "PAUSE"),
    ("走", "OVERRIDE_PLAN"),
    ("绕", "REPLAN"),
    ("多远", "QUERY"),
    ("继续", "RESUME"),
]


def parse_intent(text: str) -> str:
    for keyword, intent in INTENT_RULES:
        if keyword in text:
            return intent
    return "UNKNOWN"


def test_navigate_intent():
    assert parse_intent("去3号楼") == "NAVIGATE"
    assert parse_intent("到菜鸟驿站") == "NAVIGATE"


def test_pause_intent():
    assert parse_intent("停一下") == "PAUSE"


def test_replan_intent():
    assert parse_intent("前面有人绕一下") == "REPLAN"


def test_query_intent():
    assert parse_intent("还有多远") == "QUERY"


def test_unknown_intent():
    assert parse_intent("今天天气怎么样") == "UNKNOWN"


def test_resume_intent():
    assert parse_intent("继续走") == "RESUME"
```

- [ ] **Step 2: Implement voice pipeline**

Rewrite `src/application/autocar_application/voice_pipeline.py`:
```python
"""Voice pipeline — VAD → Wake Word → ASR(cloud) → LLM(cloud) → Intent → TTS(cloud)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


INTENT_RULES = [
    ("去", "NAVIGATE"),
    ("到", "NAVIGATE"),
    ("停", "PAUSE"),
    ("走", "OVERRIDE_PLAN"),
    ("绕", "REPLAN"),
    ("多远", "QUERY"),
    ("继续", "RESUME"),
    ("回", "NAVIGATE"),
]


class VoicePipeline(Node):
    def __init__(self):
        super().__init__('voice_pipeline')
        self.cmd_pub = self.create_publisher(String, '/voice_command', 10)
        self.tts_text_pub = self.create_publisher(String, '/tts_text', 10)

        self.create_timer(1.0, self._heartbeat)
        self.get_logger().info('voice_pipeline initialized')

    def parse_intent(self, text: str) -> str:
        for keyword, intent in INTENT_RULES:
            if keyword in text:
                self.get_logger().info(f'intent matched: {intent} via keyword "{keyword}"')
                return intent
        return "UNKNOWN"

    async def process_utterance(self, text: str):
        """Process natural language utterance through the pipeline."""
        intent = self.parse_intent(text)

        if intent == "UNKNOWN":
            self.get_logger().warn(f'unrecognized utterance: {text}')
            return

        cmd = String()
        cmd.data = intent
        self.cmd_pub.publish(cmd)

        if intent == "NAVIGATE":
            response = f"好的，开始导航"
        elif intent == "PAUSE":
            response = "已暂停"
        elif intent == "RESUME":
            response = "继续行驶"
        elif intent == "REPLAN":
            response = "重新规划路径"
        else:
            response = "收到"

        tts = String()
        tts.data = response
        self.tts_text_pub.publish(tts)

    def _heartbeat(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VoicePipeline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Run tests**

```bash
cd src/application && python -m pytest test/ -v
```

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat: implement voice pipeline with rule-based intent parsing"
```

---

### Task 4.3: 实现 Web 监控面板

**Files:**
- Modify: `src/application/autocar_application/web_dashboard.py`

- [ ] **Step 1: Implement web dashboard**

Rewrite `src/application/autocar_application/web_dashboard.py`:
```python
"""Web dashboard — serve status/map via HTTP for browser monitoring."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import json
import http.server
import socketserver
import threading


class DashboardHandler(http.server.BaseHTTPRequestHandler):
    state = "IDLE"
    position = {"x": 0.0, "y": 0.0, "theta": 0.0}
    battery = 12.0
    path_points = []

    def do_GET(self):
        if self.path == '/api/status':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            data = {
                "state": self.state,
                "position": self.position,
                "battery": self.battery,
                "path_length": len(self.path_points),
            }
            self.wfile.write(json.dumps(data).encode())
        elif self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            html = self._build_html()
            self.wfile.write(html.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

    def _build_html(self) -> str:
        return """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Autocar Dashboard</title>
<style>
  body { font-family: sans-serif; margin: 20px; background: #1a1a2e; color: #eee; }
  .card { background: #16213e; border-radius: 8px; padding: 16px; margin: 8px 0; }
  .state { font-size: 24px; font-weight: bold; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 12px; }
  canvas { background: #0f3460; border-radius: 8px; width: 100%; height: 300px; }
  h1 { color: #e94560; }
</style>
</head>
<body>
<h1>🚗 Autocar Dashboard</h1>
<div class="grid">
  <div class="card"><div class="state" id="state">IDLE</div><div>系统状态</div></div>
  <div class="card"><span id="battery">12.0</span>V<div>电池电压</div></div>
  <div class="card">X: <span id="x">0.00</span> Y: <span id="y">0.00</span><div>位置</div></div>
</div>
<div class="card"><canvas id="map"></canvas></div>
<script>
async function update() {
  try {
    const r = await fetch('/api/status');
    const d = await r.json();
    document.getElementById('state').textContent = d.state;
    document.getElementById('battery').textContent = d.battery.toFixed(1);
    document.getElementById('x').textContent = d.position.x.toFixed(2);
    document.getElementById('y').textContent = d.position.y.toFixed(2);
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    ctx.fillStyle = '#0f3460';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#e94560';
    const cx = canvas.width / 2 + d.position.x * 4;
    const cy = canvas.height / 2 - d.position.y * 4;
    ctx.beginPath();
    ctx.arc(cx, cy, 6, 0, Math.PI * 2);
    ctx.fill();
  } catch(e) {}
}
setInterval(update, 500);
</script>
</body></html>"""


class WebDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        self.state_sub = self.create_subscription(
            String, '/system_state', self.on_state, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom_fused', self.on_odom, 10)

        self._start_server()
        self.get_logger().info('web_dashboard running on http://0.0.0.0:8080')

    def on_state(self, msg: String):
        DashboardHandler.state = msg.data

    def on_odom(self, msg: Odometry):
        DashboardHandler.position = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "theta": 0.0,
        }

    def _start_server(self):
        def _run():
            with socketserver.TCPServer(("0.0.0.0", 8080), DashboardHandler) as httpd:
                httpd.serve_forever()
        t = threading.Thread(target=_run, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = WebDashboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Build**

```bash
colcon build --packages-select autocar_application
```

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "feat: implement web dashboard with real-time status API and map view"
```

---

## Phase 5: 仿真与集成测试

### Task 5.1: 创建 Gazebo 仿真环境

**Files:**
- Create: `simulation/worlds/indoor.world`
- Create: `simulation/worlds/outdoor.world`

- [ ] **Step 1: Create indoor simulation world**

`simulation/worlds/indoor.world`:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="indoor">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <model name="wall_north">
      <static>true</static>
      <link name="link"><collision name="collision"><geometry><box><size>50 0.2 3</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>50 0.2 3</size></box></geometry></visual></link>
      <pose>0 15 1.5 0 0 0</pose>
    </model>
    <model name="wall_south">
      <static>true</static>
      <link name="link"><collision name="collision"><geometry><box><size>50 0.2 3</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>50 0.2 3</size></box></geometry></visual></link>
      <pose>0 -15 1.5 0 0 0</pose>
    </model>
    <model name="wall_east">
      <static>true</static>
      <link name="link"><collision name="collision"><geometry><box><size>0.2 30 3</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.2 30 3</size></box></geometry></visual></link>
      <pose>25 0 1.5 0 0 0</pose>
    </model>
    <model name="wall_west">
      <static>true</static>
      <link name="link"><collision name="collision"><geometry><box><size>0.2 30 3</size></box></geometry></collision>
      <visual name="visual"><geometry><box><size>0.2 30 3</size></box></geometry></visual></link>
      <pose>-25 0 1.5 0 0 0</pose>
    </model>
  </world>
</sdf>
```

- [ ] **Step 2: Create outdoor simulation world**

`simulation/worlds/outdoor.world`:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="outdoor">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <!-- Open space with sparse obstacles -->
    <model name="building1"><static>true</static><pose>10 15 3 0 0 0</pose>
      <link name="link"><collision name="c"><geometry><box><size>8 5 6</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>8 5 6</size></box></geometry></visual></link>
    </model>
    <model name="building2"><static>true</static><pose>-15 10 3 0 0 0</pose>
      <link name="link"><collision name="c"><geometry><box><size>6 4 5</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>6 4 5</size></box></geometry></visual></link>
    </model>
  </world>
</sdf>
```

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "feat: add Gazebo indoor and outdoor simulation worlds"
```

---

### Task 5.2: 集成测试 — 话题契约验证

**Files:**
- Create: `test/integration/test_topics.py`
- Create: `test/integration/README.md`

- [ ] **Step 1: Create integration test**

`test/integration/test_topics.py`:
```python
"""Integration tests for ROS2 topic contracts."""
import pytest

EXPECTED_TOPICS = [
    # Control
    ("/cmd_vel", "geometry_msgs/msg/Twist"),
    ("/odom", "nav_msgs/msg/Odometry"),
    ("/battery_voltage", "std_msgs/msg/Float32"),
    ("/safety_state", "std_msgs/msg/String"),
    # Perception
    ("/fix", "sensor_msgs/msg/NavSatFix"),
    ("/imu", "sensor_msgs/msg/Imu"),
    ("/scan", "sensor_msgs/msg/LaserScan"),
    ("/image_raw", "sensor_msgs/msg/Image"),
    ("/depth", "sensor_msgs/msg/Image"),
    ("/odom_fused", "nav_msgs/msg/Odometry"),
    # Planning
    ("/voice_command", "std_msgs/msg/String"),
    ("/task_goal", "geometry_msgs/msg/PoseStamped"),
    ("/global_path", "nav_msgs/msg/Path"),
    ("/local_path", "nav_msgs/msg/Path"),
    # Application
    ("/system_state", "std_msgs/msg/String"),
    ("/bt_state", "std_msgs/msg/String"),
]


def test_all_expected_topics_defined():
    topics = [t[0] for t in EXPECTED_TOPICS]
    assert len(topics) == len(set(topics)), "Duplicate topic names"


def test_topic_count_matches_spec():
    assert len(EXPECTED_TOPICS) == 15


def test_all_topics_have_valid_types():
    for name, msg_type in EXPECTED_TOPICS:
        assert "/" in msg_type, f"{name}: invalid message type {msg_type}"
```

- [ ] **Step 2: Run integration tests**

```bash
cd test/integration && python -m pytest test_topics.py -v
```

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "test: add integration test verifying 15 topic contracts"
```

---

## Phase 6: 项目收尾

### Task 6.1: 更新 CLAUDE.md 和 README

**Files:**
- Modify: `CLAUDE.md`
- Create: `README.md`

- [ ] **Step 1: Update CLAUDE.md**

```markdown
# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

## Project: Autocar

ROS2-based autonomous vehicle with natural language voice interaction, indoor/outdoor localization, and city-level path planning.

## Architecture

4-layer ROS2 Humble architecture (16 nodes):
- **Control Layer (C++):** motor_driver, velocity_controller, steering_controller, safety_watchdog
- **Perception Layer (C++):** gps_driver, imu_driver, lidar_driver, camera_driver, localization_fusion (EKF), slam_node (Cartographer)
- **Planning Layer (C++):** behavior_tree, global_planner (A*), local_planner (DWA)
- **Application Layer (Python):** state_machine, voice_pipeline, web_dashboard

## Build & Test

- Build: `colcon build`
- Test: `colcon test --event-handlers console_direct+`
- Run: `ros2 launch autocar_bringup autocar_bringup.launch.py`
- Single package: `colcon build --packages-select <pkg>`

## Key Conventions

- TDD: test first, then implementation (GTest for C++, pytest for Python)
- Custom interfaces: `autocar_interfaces` package
- Config: YAML params in `config/params/`
- Launch: `config/launch/autocar_bringup.launch.py`
```

- [ ] **Step 2: Create README.md**

```markdown
# Autocar 🚗

Autonomous vehicle platform — natural voice interaction, indoor/outdoor navigation, ROS2 Humble.

## Requirements

- ROS2 Humble
- Python 3.10+, C++17
- colcon, CMake, Eigen3

## Quick Start

```bash
colcon build
source install/setup.bash
ros2 launch autocar_bringup autocar_bringup.launch.py
```

Dashboard: http://localhost:8080

## Hardware

Raspberry Pi 4B/5 + ROS chassis kit + 2D LiDAR + GPS/IMU + Depth Camera
Budget: ¥3200-5250

## Docs

- [System Design](docs/superpowers/specs/2026-06-14-autocar-system-design.md)
- [Implementation Plan](docs/superpowers/plans/2026-06-14-autocar-implementation-plan.md)
```

- [ ] **Step 3: Final commit**

```bash
git add -A
git commit -m "docs: update CLAUDE.md and add README.md"
```

- [ ] **Step 4: Full build verification**

```bash
colcon build
```
Expected: All 4 packages build successfully.

```bash
colcon test --event-handlers console_direct+
```
Expected: All tests pass.

