"""Launch Gazebo Ignition with autocar_car robot model and ros_ign_bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = 'autocar_simulation'
    sim_dir = get_package_share_directory(sim_pkg)

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

    world_file = os.path.join(sim_dir, 'worlds', '{}.sdf')

    # Gazebo server only (headless)
    gz_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '--world-name', world_name,
             world_file.format('test_obstacle')],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': os.path.join(sim_dir, 'models')},
        condition=IfCondition(headless),
    )

    # Gazebo with GUI
    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '--world-name', world_name,
             world_file.format('test_obstacle')],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': os.path.join(sim_dir, 'models')},
        condition=UnlessCondition(headless),
    )

    # Spawn robot
    robot_sdf = os.path.join(sim_dir, 'models', 'autocar_car', 'model.sdf')
    spawn_robot = ExecuteProcess(
        cmd=[
            'ign', 'service', '-s', '/world/test_obstacle/create',
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '10',
            '--req', 'sdf_filename: "{}"'.format(robot_sdf),
        ],
        output='screen',
    )

    # ── ros_ign_bridge: Bridge Ignition Transport <-> ROS2 ──
    # Direction:  [ = ign -> ros,  ] = ros -> ign,  <-> bidir
    # LiDAR scan: Ignition -> ROS2
    scan_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_scan',
        arguments=['/model/autocar_car/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # IMU: Ignition -> ROS2
    imu_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=['/model/autocar_car/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # GPS: Ignition -> ROS2
    gps_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_gps',
        arguments=['/model/autocar_car/gps@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSatFix'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Odometry: Ignition -> ROS2
    odom_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=['/model/autocar_car/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # cmd_vel: ROS2 -> Ignition (planning output -> Gazebo)
    cmd_vel_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        arguments=['/model/autocar_car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Depth image: Ignition -> ROS2
    depth_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_depth',
        arguments=['/model/autocar_car/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # TF: Ignition -> ROS2 (DiffDrive publishes odom -> base_link)
    tf_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_tf',
        arguments=['/model/autocar_car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Static TF: map -> odom
    tf_static_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([
        world_arg, noise_arg, headless_arg,
        gz_server, gz_gui,
        spawn_robot,
        scan_bridge, imu_bridge, gps_bridge, odom_bridge,
        cmd_vel_bridge, depth_bridge, tf_bridge,
        tf_static_node,
    ])
