"""Launch Gazebo Ignition with autocar_car robot model and ros_ign_bridge.

Usage:
  ros2 launch autocar_simulation sim_bringup.launch.py world:=test_obstacle
  ros2 launch autocar_simulation sim_bringup.launch.py world:=city_street headless:=false
  ros2 launch autocar_simulation sim_bringup.launch.py world:=test_obstacle headless:=true
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_world_path(context, sim_dir):
    """Resolve world file path at launch time from the world argument."""
    world_name = LaunchConfiguration('world').perform(context)
    world_path = os.path.join(sim_dir, 'worlds', world_name + '.sdf')
    if not os.path.exists(world_path):
        # Fallback to test_obstacle
        world_path = os.path.join(sim_dir, 'worlds', 'test_obstacle.sdf')
    return world_path


def _launch_gazebo(context, *args, **kwargs):
    """Create Gazebo launch actions with resolved paths."""
    sim_dir = get_package_share_directory('autocar_simulation')
    world_path = _build_world_path(context, sim_dir)
    headless = LaunchConfiguration('headless').perform(context) == 'true'

    cmd = ['ign', 'gazebo', '-r']
    if headless:
        cmd += ['-s']
    cmd += [world_path]

    return [ExecuteProcess(
        cmd=cmd,
        output='screen',
        additional_env={
            'IGN_GAZEBO_RESOURCE_PATH': os.path.join(sim_dir, 'models'),
            'DISPLAY': ':0',
        },
    )]


def _spawn_robot(context, *args, **kwargs):
    """Spawn the autocar_car model into the running Gazebo instance."""
    sim_dir = get_package_share_directory('autocar_simulation')
    world_name = LaunchConfiguration('world').perform(context) or 'test_obstacle'
    robot_sdf = os.path.join(sim_dir, 'models', 'autocar_car', 'model.sdf')
    spawn_script = os.path.join(sim_dir, 'launch', 'spawn_robot.py')

    return [ExecuteProcess(
        cmd=[sys.executable, spawn_script, world_name, robot_sdf],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='test_obstacle',
                              description='World file name (without .sdf)'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without GUI'),
        DeclareLaunchArgument('noise', default_value='true',
                              description='Enable sensor noise'),

        # Gazebo server+gui (uses OpaqueFunction for runtime path resolution)
        OpaqueFunction(function=_launch_gazebo),

        # Spawn robot into the world
        OpaqueFunction(function=_spawn_robot),

        # ── ros_ign_bridge: Ignition Transport <-> ROS2 ──
        # Note: ros_ign_bridge shows deprecation warning, redirects to ros_gz_bridge
        # Direction:  [ = ign->ros   ] = ros->ign   <-> = bidirectional

        # Sensor topics: Ignition -> ROS2 (remapped to match ROS node expectations)
        Node(package='ros_ign_bridge', executable='parameter_bridge',
             name='bridge_scan',
             arguments=['/model/autocar_car/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
             parameters=[{'use_sim_time': True}],
             remappings=[('/model/autocar_car/scan', '/scan')]),
        Node(package='ros_ign_bridge', executable='parameter_bridge',
             name='bridge_imu',
             arguments=['/model/autocar_car/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
             parameters=[{'use_sim_time': True}],
             remappings=[('/model/autocar_car/imu', '/imu')]),
        Node(package='ros_ign_bridge', executable='parameter_bridge',
             name='bridge_odom',
             arguments=['/model/autocar_car/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
             parameters=[{'use_sim_time': True}],
             remappings=[('/model/autocar_car/odom', '/odom')]),
        Node(package='ros_ign_bridge', executable='parameter_bridge',
             name='bridge_depth',
             arguments=['/model/autocar_car/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image'],
             parameters=[{'use_sim_time': True}],
             remappings=[('/model/autocar_car/depth_image', '/depth')]),

        # cmd_vel: ROS2 -> Ignition (planning/cmd_vel -> Gazebo DiffDrive)
        Node(package='ros_ign_bridge', executable='parameter_bridge',
             name='bridge_cmd_vel',
             arguments=['/model/autocar_car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
             parameters=[{'use_sim_time': True}],
             remappings=[('/model/autocar_car/cmd_vel', '/cmd_vel')]),

        # Static TF: map -> odom
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='map_to_odom_static',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
    ])
