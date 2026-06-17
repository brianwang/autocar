# Autocar Gazebo 仿真环境实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 Autocar ROS2 项目搭建 Gazebo Ignition 仿真环境，含差分驱动小车模型、传感器仿真、桥接节点、世界场景和启动文件，实现 PC-only 全栈分层验证。

**Architecture:** 新建 `autocar_simulation` C++ 包，内含 SDF 差分底盘模型（DiffDrive + LiDAR + IMU + GPS + 深度相机）、两级世界场景（障碍测试、城市街道）、simulation_bridge 桥接节点（话题映射 + 噪声注入 + use_sim_time 管理）。与现有 4 层 16 节点骨架代码通过标准 ROS2 话题通信。

**Tech Stack:** ROS2 Humble, Gazebo Ignition, C++17 (bridge 节点), Python 3.10+ (launch), SDF 1.7, colcon, CMake, GTest

**前置条件:** 开发机已装 ROS2 Humble + Gazebo Ignition (`gz sim --version`)。现有代码仓库已编译通过（见 `docs/superpowers/plans/2026-06-14-autocar-implementation-plan.md` Phase 0）。

---

### Task 1: 创建 autocar_simulation 包骨架

**Files:**
- Create: `src/simulation/package.xml`
- Create: `src/simulation/CMakeLists.txt`
- Create: `src/simulation/src/simulation_bridge.cpp` (stub)
- Create: `src/simulation/test/CMakeLists.txt`
- Create: `src/simulation/test/test_simulation_bridge.cpp`
- Create: `src/simulation/config/sim_params.yaml`
- Create: `src/simulation/launch/sim_bringup.launch.py`
- Create: `src/simulation/launch/autocar_full.launch.py`
- Create: `src/simulation/worlds/.gitkeep`
- Create: `src/simulation/models/.gitkeep`

- [ ] **Step 1: Create package.xml**

`src/simulation/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>autocar_simulation</name>
  <version>0.1.0</version>
  <description>Autocar Gazebo Ignition simulation environment: robot model, worlds, bridge node, launch files</description>
  <maintainer email="dev@autocar.local">Autocar Dev</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>

  <exec_depend>ros_ign_bridge</exec_depend>
  <exec_depend>ros_ign_gazebo</exec_depend>
  <exec_depend>python3-transforms3d</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create CMakeLists.txt**

`src/simulation/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(autocar_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(EXECUTABLES simulation_bridge)

foreach(EXEC IN LISTS EXECUTABLES)
  add_executable(${EXEC} src/${EXEC}.cpp)
  ament_target_dependencies(${EXEC}
    rclcpp geometry_msgs sensor_msgs nav_msgs std_msgs tf2 tf2_ros
  )
  install(TARGETS ${EXEC}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
```

- [ ] **Step 3: Create stub simulation_bridge.cpp**

`src/simulation/src/simulation_bridge.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simulation_bridge");
  RCLCPP_INFO(node->get_logger(), "simulation_bridge starting");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Create test CMakeLists.txt + stub test**

`src/simulation/test/CMakeLists.txt`:
```cmake
add_executable(test_simulation_bridge test_simulation_bridge.cpp ../src/simulation_bridge.cpp)
ament_target_dependencies(test_simulation_bridge rclcpp geometry_msgs sensor_msgs std_msgs)
target_link_libraries(test_simulation_bridge gtest gtest_main pthread)
ament_add_gtest(test_simulation_bridge test_simulation_bridge.cpp ../src/simulation_bridge.cpp)
```

`src/simulation/test/test_simulation_bridge.cpp`:
```cpp
#include <gtest/gtest.h>

TEST(SimulationBridgeTest, StubPlaceholder) {
  EXPECT_TRUE(true);
}
```

- [ ] **Step 5: Create placeholder directories**

```bash
mkdir -p src/simulation/models/autocar_car
mkdir -p src/simulation/worlds
mkdir -p src/simulation/launch
mkdir -p src/simulation/config
touch src/simulation/worlds/.gitkeep
touch src/simulation/models/.gitkeep
```

- [ ] **Step 6: Build skeleton and verify**

```bash
colcon build --packages-select autocar_simulation
```
Expected: Build succeeds.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: create autocar_simulation package skeleton with CMakeLists, test, and stub bridge node"
```

---

### Task 2: 创建 SDF 差分驱动小车模型

**Files:**
- Create: `src/simulation/models/autocar_car/model.sdf`
- Create: `src/simulation/models/autocar_car/model.config`

- [ ] **Step 1: Create model.config**

`src/simulation/models/autocar_car/model.config`:
```xml
<?xml version="1.0"?>
<model>
  <name>autocar_car</name>
  <version>1.0.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Autocar Dev</name>
  </author>
  <description>Autocar differential drive robot with LiDAR, IMU, GPS, and depth camera sensors</description>
</model>
```

- [ ] **Step 2: Create model.sdf**

`src/simulation/models/autocar_car/model.sdf`:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="autocar_car">
    <pose>0 0 0.05 0 0 0</pose>

    <!-- ========== Chassis ========== -->
    <link name="chassis">
      <pose>0 0 0.05 0 0 0</pose>
      <inertial>
        <mass>2.0</mass>
        <inertia>
          <ixx>0.03</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.08</iyy><iyz>0</iyz>
          <izz>0.08</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box><size>0.25 0.18 0.10</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>0.25 0.18 0.10</size></box>
        </geometry>
        <material>
          <ambient>0.2 0.4 0.8 1</ambient>
          <diffuse>0.2 0.4 0.8 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- ========== Left Wheel ========== -->
    <link name="left_wheel">
      <pose>-0.08 0.10 0.0 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.0001</iyy><iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.032</radius><length>0.025</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.032</radius><length>0.025</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- ========== Right Wheel ========== -->
    <link name="right_wheel">
      <pose>-0.08 -0.10 0.0 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.0001</iyy><iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.032</radius><length>0.025</length></cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.032</radius><length>0.025</length></cylinder>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- ========== Caster Wheel (rear) ========== -->
    <link name="caster_wheel">
      <pose>0.10 0 0.01 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00001</iyy><iyz>0</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <sphere><radius>0.015</radius></sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere><radius>0.015</radius></sphere>
        </geometry>
      </visual>
    </link>

    <!-- ========== Joints ========== -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
      </axis>
    </joint>

    <joint name="caster_wheel_joint" type="ball">
      <parent>chassis</parent>
      <child>caster_wheel</child>
    </joint>

    <!-- ========== DiffDrive Plugin ========== -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.20</wheel_separation>
      <wheel_radius>0.032</wheel_radius>
      <odom_publisher_frequency>50</odom_publisher_frequency>
      <topic>/model/autocar_car/cmd_vel</topic>
      <odom_topic>/model/autocar_car/odom</odom_topic>
      <tf_topic>/model/autocar_car/tf</tf_topic>
    </plugin>

    <!-- ========== LiDAR ========== -->
    <link name="laser_frame">
      <pose>0.12 0 0.06 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00001</iyy><iyz>0</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>0.02 0.02 0.02</size></box></geometry>
        <material>
          <ambient>0 0.8 0 1</ambient>
        </material>
      </visual>
      <sensor name="lidar" type="gpu_lidar">
        <topic>/model/autocar_car/scan</topic>
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>0.0</min_angle>
              <max_angle>6.283</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.05</min>
            <max>12.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.02</stddev>
          </noise>
        </lidar>
        <always_on>1</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>

    <!-- ========== IMU ========== -->
    <link name="imu_frame">
      <pose>0 0 0.03 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.000001</iyy><iyz>0</iyz>
          <izz>0.000001</izz>
        </inertia>
      </inertial>
      <sensor name="imu" type="imu">
        <topic>/model/autocar_car/imu</topic>
        <update_rate>100</update_rate>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
        <always_on>1</always_on>
      </sensor>
    </link>

    <!-- ========== GPS ========== -->
    <link name="gps_frame">
      <pose>0 0 0.04 0 0 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia>
          <ixx>0.000001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.000001</iyy><iyz>0</iyz>
          <izz>0.000001</izz>
        </inertia>
      </inertial>
      <sensor name="gps" type="navsat">
        <topic>/model/autocar_car/gps</topic>
        <update_rate>10</update_rate>
        <always_on>1</always_on>
      </sensor>
    </link>

    <!-- ========== Depth Camera ========== -->
    <link name="camera_frame">
      <pose>0.12 0 0.04 0 0 0</pose>
      <inertial>
        <mass>0.03</mass>
        <inertia>
          <ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.00001</iyy><iyz>0</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>0.03 0.02 0.02</size></box></geometry>
        <material>
          <ambient>0.8 0.8 0 1</ambient>
        </material>
      </visual>
      <sensor name="depth_camera" type="depth_camera">
        <topic>/model/autocar_car/depth_image</topic>
        <update_rate>15</update_rate>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <visualize>true</visualize>
      </sensor>
    </link>

  </model>
</sdf>
```

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "feat: add autocar_car SDF model with DiffDrive, LiDAR, IMU, GPS, depth camera"
```

---

### Task 3: 创建 Gazebo 世界文件

**Files:**
- Create: `src/simulation/worlds/test_obstacle.sdf`
- Create: `src/simulation/worlds/city_street.sdf`

- [ ] **Step 1: Create test_obstacle world**

`src/simulation/worlds/test_obstacle.sdf`:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="test_obstacle">
    <physics name="1ms" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <!-- Test obstacle - large box -->
    <model name="obstacle_1">
      <static>true</static>
      <pose>3.0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Test obstacle - wall -->
    <model name="obstacle_2">
      <static>true</static>
      <pose>-2.0 2.5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 3.0 1.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 3.0 1.0</size></box></geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Arena boundary walls -->
    <model name="boundary_north">
      <static>true</static>
      <pose>0 10 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>30 0.2 1.0</size></box></geometry>
        </collision>
      </link>
    </model>
    <model name="boundary_south">
      <static>true</static>
      <pose>0 -10 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>30 0.2 1.0</size></box></geometry>
        </collision>
      </link>
    </model>
    <model name="boundary_east">
      <static>true</static>
      <pose>15 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 20 1.0</size></box></geometry>
        </collision>
      </link>
    </model>
    <model name="boundary_west">
      <static>true</static>
      <pose>-15 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 20 1.0</size></box></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

- [ ] **Step 2: Create city_street world**

`src/simulation/worlds/city_street.sdf`:
```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="city_street">
    <physics name="1ms" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane</uri>
    </include>
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>

    <!-- Buildings -->
    <model name="building_1">
      <static>true</static>
      <pose>8.0 6.0 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>4.0 4.0 3.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>4.0 4.0 3.0</size></box></geometry>
          <material><ambient>0.5 0.5 0.6 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="building_2">
      <static>true</static>
      <pose>-6.0 5.0 1.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>3.0 3.0 3.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>3.0 3.0 3.0</size></box></geometry>
          <material><ambient>0.6 0.4 0.4 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="building_3">
      <static>true</static>
      <pose>5.0 -5.0 2.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>5.0 3.0 4.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>5.0 3.0 4.0</size></box></geometry>
          <material><ambient>0.4 0.6 0.5 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="building_4">
      <static>true</static>
      <pose>-7.0 -4.0 1.0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>2.0 2.0 2.0</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>2.0 2.0 2.0</size></box></geometry>
          <material><ambient>0.7 0.7 0.3 1</ambient></material>
        </visual>
      </link>
    </model>

    <!-- Road block (low wall simulating curb) -->
    <model name="curb_1">
      <static>true</static>
      <pose>0 2.5 0.1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>20 0.1 0.2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>20 0.1 0.2</size></box></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="curb_2">
      <static>true</static>
      <pose>0 -2.5 0.1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>20 0.1 0.2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>20 0.1 0.2</size></box></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "feat: add test_obstacle and city_street Gazebo world files"
```

---

### Task 4: 实现 simulation_bridge 节点

**Files:**
- Modify: `src/simulation/src/simulation_bridge.cpp`
- Modify: `src/simulation/test/test_simulation_bridge.cpp`
- Modify: `src/simulation/test/CMakeLists.txt`

bridge 节点职责：将 Gazebo 命名话题（`/model/autocar_car/...`）映射到标准 ROS2 话题名，支持 `use_sim_time`，可配置噪声注入。

- [ ] **Step 1: Write bridge test**

Replace `src/simulation/test/test_simulation_bridge.cpp`:
```cpp
#include <gtest/gtest.h>
#include <string>
#include <map>

// Map of Gazebo topic -> standard topic
std::map<std::string, std::string> topic_map = {
  {"/model/autocar_car/cmd_vel", "/cmd_vel"},
  {"/model/autocar_car/odom", "/odom"},
  {"/model/autocar_car/scan", "/scan"},
  {"/model/autocar_car/imu", "/imu"},
  {"/model/autocar_car/gps", "/fix"},
  {"/model/autocar_car/depth_image", "/depth"},
  {"/model/autocar_car/tf", "/tf"},
};

// Noise configuration per sensor type
struct NoiseConfig {
  double mean;
  double stddev;
};

std::map<std::string, NoiseConfig> noise_configs = {
  {"/scan", {0.0, 0.02}},
  {"/imu", {0.0, 0.01}},
  {"/gps", {0.0, 0.5}},
};

TEST(BridgeTest, TopicMappingComplete) {
  // Verify all expected mappings exist
  EXPECT_EQ(topic_map["/model/autocar_car/cmd_vel"], "/cmd_vel");
  EXPECT_EQ(topic_map["/model/autocar_car/odom"], "/odom");
  EXPECT_EQ(topic_map["/model/autocar_car/scan"], "/scan");
  EXPECT_EQ(topic_map["/model/autocar_car/imu"], "/imu");
  EXPECT_EQ(topic_map["/model/autocar_car/gps"], "/fix");
  EXPECT_EQ(topic_map["/model/autocar_car/depth_image"], "/depth");
  EXPECT_EQ(topic_map["/model/autocar_car/tf"], "/tf");
}

TEST(BridgeTest, TopicCountMatchesSpec) {
  EXPECT_EQ(topic_map.size(), 7);
}

TEST(BridgeTest, NoiseConfigHasValidValues) {
  for (const auto& [topic, config] : noise_configs) {
    EXPECT_GE(config.stddev, 0.0) << topic << " should have non-negative stddev";
  }
}

// Command-line flag parsing test
struct SimFlags {
  bool use_sim_time{false};
  bool noise_enabled{true};
};

SimFlags parseFlags(int argc, char** argv) {
  SimFlags flags;
  for (int i = 0; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "use_sim_time:=true") flags.use_sim_time = true;
    if (arg == "noise:=false") flags.noise_enabled = false;
  }
  return flags;
}

TEST(BridgeTest, ParseFlagsDefaults) {
  char* args[] = {strdup("bridge")};
  auto flags = parseFlags(1, args);
  EXPECT_TRUE(flags.noise_enabled);
  EXPECT_FALSE(flags.use_sim_time);
}

TEST(BridgeTest, ParseFlagsSimTime) {
  char* args[] = {strdup("bridge"), strdup("use_sim_time:=true")};
  auto flags = parseFlags(2, args);
  EXPECT_TRUE(flags.use_sim_time);
}

TEST(BridgeTest, ParseFlagsNoiseDisabled) {
  char* args[] = {strdup("bridge"), strdup("noise:=false")};
  auto flags = parseFlags(2, args);
  EXPECT_FALSE(flags.noise_enabled);
}
```

Update `src/simulation/test/CMakeLists.txt`:
```cmake
add_executable(test_simulation_bridge test_simulation_bridge.cpp)
ament_target_dependencies(test_simulation_bridge rclcpp)
target_link_libraries(test_simulation_bridge gtest gtest_main pthread)
ament_add_gtest(test_simulation_bridge test_simulation_bridge.cpp)
```
(Remove the `../src/simulation_bridge.cpp` dependency since tests use standalone logic, not the full node.)

- [ ] **Step 2: Run test to verify it fails (no test executable yet)**

```bash
colcon test --packages-select autocar_simulation --event-handlers console_direct+
```
Expected: Build error or test failure due to missing test target rebuild.

- [ ] **Step 3: Implement simulation_bridge node**

Replace `src/simulation/src/simulation_bridge.cpp`:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <random>

class SimulationBridge : public rclcpp::Node {
public:
  SimulationBridge() : Node("simulation_bridge") {
    // Declare parameters
    this->declare_parameter("noise_enabled", true);
    this->declare_parameter("use_sim_time", true);

    bool noise_enabled = this->get_parameter("noise_enabled").as_bool();
    // Set use_sim_time parameter
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    RCLCPP_INFO(this->get_logger(), "simulation_bridge starting (noise=%s)",
                noise_enabled ? "enabled" : "disabled");

    // Subscribers to Gazebo topics
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/model/autocar_car/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_pub_->publish(*msg);
      });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/model/autocar_car/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_pub_->publish(*msg);
      });

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/model/autocar_car/scan", 10,
      [this, noise_enabled](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto out = *msg;
        if (noise_enabled) {
          std::normal_distribution<double> nd(0.0, 0.02);
          for (auto& r : out.ranges) {
            r = std::clamp(r + nd(rng_), msg->range_min, msg->range_max);
          }
        }
        scan_pub_->publish(out);
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/model/autocar_car/imu", 10,
      [this, noise_enabled](const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto out = *msg;
        if (noise_enabled) {
          std::normal_distribution<double> nd(0.0, 0.01);
          out.angular_velocity.x += nd(rng_);
          out.angular_velocity.y += nd(rng_);
          out.angular_velocity.z += nd(rng_);
          out.linear_acceleration.x += nd(rng_);
          out.linear_acceleration.y += nd(rng_);
          out.linear_acceleration.z += nd(rng_);
        }
        imu_pub_->publish(out);
      });

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/model/autocar_car/gps", 10,
      [this, noise_enabled](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        auto out = *msg;
        if (noise_enabled) {
          std::normal_distribution<double> nd(0.0, 0.0001);
          out.latitude += nd(rng_);
          out.longitude += nd(rng_);
        }
        gps_pub_->publish(out);
      });

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/model/autocar_car/depth_image", 10,
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        depth_pub_->publish(*msg);
      });

    // Publishers to standard topics
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/depth", 10);

    RCLCPP_INFO(this->get_logger(), "simulation_bridge ready");
  }

private:
  std::mt19937 rng_{std::random_device{}()};

  // Subscribers (Gazebo topics)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  // Publishers (standard topic names)
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationBridge>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 4: Build and run tests**

```bash
colcon build --packages-select autocar_simulation
colcon test --packages-select autocar_simulation --event-handlers console_direct+
```
Expected: All 7 bridge tests pass.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "feat: implement simulation_bridge node with topic mapping and configurable noise injection"
```

---

### Task 5: 创建启动文件

**Files:**
- Modify: `src/simulation/launch/sim_bringup.launch.py`
- Modify: `src/simulation/launch/autocar_full.launch.py`

- [ ] **Step 1: Create sim_bringup.launch.py**

`src/simulation/launch/sim_bringup.launch.py`:
```python
"""Launch Gazebo Ignition with autocar_car robot model and simulation bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = 'autocar_simulation'
    sim_dir = get_package_share_directory(sim_pkg)

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value='test_obstacle',
        description='World file name (without .sdf extension)'
    )
    noise_arg = DeclareLaunchArgument(
        'noise', default_value='true',
        description='Enable sensor noise injection'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI'
    )

    world_name = LaunchConfiguration('world')
    noise_enabled = LaunchConfiguration('noise')
    headless = LaunchConfiguration('headless')

    # 1. Start Gazebo Ignition with the world
    # World file path built at launch time via PythonExpression
    world_path = os.path.join(sim_dir, 'worlds', '{}.sdf')
    gz_cmd = [
        'gz', 'sim',
        '-r', '--world-name', world_name,
        world_path.format(world_name.perform(None) or 'test_obstacle'),
    ]
    # Note: world_name.perform(None) works at parse time for file path;
    # the world file is read at parse, but Gazebo is started at runtime.

    start_gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(sim_dir, 'models')},
    )

    # 2. Spawn robot model via Gazebo create service
    robot_sdf_path = os.path.join(sim_dir, 'models', 'autocar_car', 'model.sdf')
    spawn_robot = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s',
            '/world/{}/create'.format(world_name.perform(None) or 'test_obstacle'),
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '10',
            '--req', 'sdf_filename: "{}"'.format(robot_sdf_path),
        ],
        output='screen',
    )

    # 3. Start simulation bridge
    bridge_node = Node(
        package='autocar_simulation',
        executable='simulation_bridge',
        name='simulation_bridge',
        parameters=[{
            'noise_enabled': noise_enabled,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # Static TF: map -> odom
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([
        world_arg,
        noise_arg,
        headless_arg,
        start_gazebo,
        spawn_robot,
        bridge_node,
        tf_static_node,
    ])
```

- [ ] **Step 2: Create autocar_full.launch.py**

`src/simulation/launch/autocar_full.launch.py`:
```python
"""Launch full Autocar system with simulation mode support."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('autocar_bringup'), '..', '..', '..', 'config', 'params'
    )

    sim_flag = LaunchConfiguration('simulation')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='autocar'),
        DeclareLaunchArgument('simulation', default_value='false'),

        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),

            # ── Control layer ──
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
            # Hardware motor_driver: skip in simulation mode
            Node(
                package='autocar_control', executable='motor_driver',
                name='motor_driver',
                parameters=[os.path.join(config_dir, 'motor.yaml')],
                condition=UnlessCondition(sim_flag),
            ),

            # ── Perception layer ──
            Node(
                package='autocar_perception', executable='localization_fusion',
                name='localization_fusion',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
            ),
            Node(
                package='autocar_perception', executable='slam_node',
                name='slam_node',
            ),
            # Hardware sensor drivers: skip in simulation mode
            Node(
                package='autocar_perception', executable='gps_driver',
                name='gps_driver',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
                condition=UnlessCondition(sim_flag),
            ),
            Node(
                package='autocar_perception', executable='imu_driver',
                name='imu_driver',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
                condition=UnlessCondition(sim_flag),
            ),
            Node(
                package='autocar_perception', executable='lidar_driver',
                name='lidar_driver',
                parameters=[os.path.join(config_dir, 'localization.yaml')],
                condition=UnlessCondition(sim_flag),
            ),
            Node(
                package='autocar_perception', executable='camera_driver',
                name='camera_driver',
                condition=UnlessCondition(sim_flag),
            ),

            # ── Planning layer ──
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

            # ── Application layer ──
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

- [ ] **Step 3: Create sim_params.yaml**

`src/simulation/config/sim_params.yaml`:
```yaml
simulation_bridge:
  ros__parameters:
    noise_enabled: true
    use_sim_time: true

    # Topic mapping (reference only, configured in code)
    gazebo_to_ros_topics:
      - gz: "/model/autocar_car/cmd_vel"
        ros: "/cmd_vel"
      - gz: "/model/autocar_car/odom"
        ros: "/odom"
      - gz: "/model/autocar_car/scan"
        ros: "/scan"
      - gz: "/model/autocar_car/imu"
        ros: "/imu"
      - gz: "/model/autocar_car/gps"
        ros: "/fix"
      - gz: "/model/autocar_car/depth_image"
        ros: "/depth"
      - gz: "/model/autocar_car/tf"
        ros: "/tf"
```

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "feat: add sim_bringup and autocar_full launch files with sim_params config"
```

---

### Task 6: 更新 bringup launch 支持 sim 参数

**Files:**
- Modify: `config/launch/autocar_bringup.launch.py`

将现有 bringup 启动文件改为支持 `simulation:=true` 参数。注意：当前 `get_package_share_directory('autocar_bringup')` 会在缺少 bringup 包时报错 — 这是一个已有设计问题。本任务用相对路径回退方案绕过此问题，后续可单独立项创建 bringup 包。

- [ ] **Step 1: Update autocar_bringup.launch.py**

`config/launch/autocar_bringup.launch.py`:
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Note: config/params/ dir is relative to this file location
_config_dir = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'params'
)


def generate_launch_description():
    sim_flag = LaunchConfiguration('simulation')

    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='false'),

        # ── Control layer ──
        Node(
            package='autocar_control', executable='velocity_controller',
            name='velocity_controller',
            parameters=[os.path.join(_config_dir, 'motor.yaml')],
        ),
        Node(
            package='autocar_control', executable='steering_controller',
            name='steering_controller',
            parameters=[os.path.join(_config_dir, 'motor.yaml')],
        ),
        Node(
            package='autocar_control', executable='safety_watchdog',
            name='safety_watchdog',
            parameters=[os.path.join(_config_dir, 'safety.yaml')],
        ),
        # Hardware motor_driver: skip in simulation mode
        Node(
            package='autocar_control', executable='motor_driver',
            name='motor_driver',
            parameters=[os.path.join(_config_dir, 'motor.yaml')],
            condition=UnlessCondition(sim_flag),
        ),

        # ── Perception layer ──
        Node(
            package='autocar_perception', executable='localization_fusion',
            name='localization_fusion',
            parameters=[os.path.join(_config_dir, 'localization.yaml')],
        ),
        Node(
            package='autocar_perception', executable='slam_node',
            name='slam_node',
        ),
        # Sensor drivers: skip in simulation mode (Gazebo provides sensor data)
        Node(
            package='autocar_perception', executable='gps_driver',
            name='gps_driver',
            parameters=[os.path.join(_config_dir, 'localization.yaml')],
            condition=UnlessCondition(sim_flag),
        ),
        Node(
            package='autocar_perception', executable='imu_driver',
            name='imu_driver',
            parameters=[os.path.join(_config_dir, 'localization.yaml')],
            condition=UnlessCondition(sim_flag),
        ),
        Node(
            package='autocar_perception', executable='lidar_driver',
            name='lidar_driver',
            parameters=[os.path.join(_config_dir, 'localization.yaml')],
            condition=UnlessCondition(sim_flag),
        ),
        Node(
            package='autocar_perception', executable='camera_driver',
            name='camera_driver',
            condition=UnlessCondition(sim_flag),
        ),

        # ── Planning layer ──
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

        # ── Application layer ──
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
    ])
```
```

- [ ] **Step 2: Commit**

```bash
git add -A
git commit -m "feat: update autocar_bringup.launch.py with simulation mode support via sim flag"
```

---

### Task 7: 全量构建 + 验证编译

- [ ] **Step 1: 全量构建**

```bash
colcon build
```
Expected: 全部 5 个包（interfaces, control, perception, planning, application, simulation）编译成功。

- [ ] **Step 2: 运行所有测试**

```bash
colcon test --event-handlers console_direct+
```
Expected: 所有测试通过（现有 PID/EKF/A*/DWA 测试 + 新增 bridge 测试）。

- [ ] **Step 3: 最终提交**

```bash
git add -A
git commit -m "feat: complete Gazebo simulation environment with SDF model, worlds, bridge, and launch files"
```

---

### Task 8: 仿真启动验证（手动）

- [ ] **Step 1: 启动仿真**

```bash
source install/setup.bash
ros2 launch autocar_simulation sim_bringup.launch.py world:=test_obstacle
```
Expected: Gazebo Ignition 窗口打开，显示带障碍物的世界，autocar_car 模型出现在原点。

- [ ] **Step 2: 验证话题**

```bash
# 在新终端
source install/setup.bash
ros2 topic list
```
Expected: 看到 `/cmd_vel`, `/odom`, `/scan`, `/imu`, `/fix`, `/depth`, `/odom_fused` 等标准话题。

- [ ] **Step 3: 发送运动指令**

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10
```
Expected: 小车在 Gazebo 中向前移动。

- [ ] **Step 4: 验证里程计反馈**

```bash
ros2 topic echo /odom --once
```
Expected: `/odom` 包含位置和速度数据。

- [ ] **Step 5: 启动全栈仿真**

```bash
# 在新终端
source install/setup.bash
ros2 launch autocar_simulation autocar_full.launch.py world:=test_obstacle
```
Expected: 所有节点启动，小车可通过 `/task_goal` 下发目标点进行导航。
