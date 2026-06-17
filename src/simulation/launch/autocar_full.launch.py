"""Launch full Autocar system with Gazebo simulation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = 'autocar_simulation'
    sim_dir = get_package_share_directory(sim_pkg)

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value='city_street',
        description='World file name'
    )
    noise_arg = DeclareLaunchArgument(
        'noise', default_value='true',
        description='Enable sensor noise'
    )

    world_name = LaunchConfiguration('world')
    noise_enabled = LaunchConfiguration('noise')

    # Include simulation bringup
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(sim_dir, 'launch', 'sim_bringup.launch.py')
        ]),
        launch_arguments={
            'world': world_name,
            'noise': noise_enabled,
            'headless': 'false',
        }.items(),
    )

    # Control layer (in simulation mode, skip motor_driver and sensor drivers)
    # These nodes connect to topics published by simulation_bridge
    control_nodes = [
        Node(
            package='autocar_control', executable='velocity_controller',
            name='velocity_controller',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='autocar_control', executable='steering_controller',
            name='steering_controller',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='autocar_control', executable='safety_watchdog',
            name='safety_watchdog',
            parameters=[{'use_sim_time': True}],
        ),
    ]

    # Perception layer (only EKF fusion + SLAM, no sensor drivers)
    perception_nodes = [
        Node(
            package='autocar_perception', executable='localization_fusion',
            name='localization_fusion',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='autocar_perception', executable='slam_node',
            name='slam_node',
            parameters=[{'use_sim_time': True}],
        ),
    ]

    # Planning layer
    planning_nodes = [
        Node(
            package='autocar_planning', executable='global_planner',
            name='global_planner',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='autocar_planning', executable='local_planner',
            name='local_planner',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='autocar_planning', executable='behavior_tree',
            name='behavior_tree',
            parameters=[{'use_sim_time': True}],
        ),
    ]

    # Application layer
    app_nodes = [
        Node(
            package='autocar_application', executable='state_machine',
            name='state_machine',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='autocar_application', executable='web_dashboard',
            name='web_dashboard',
            parameters=[{'use_sim_time': True}],
        ),
    ]

    return LaunchDescription([
        world_arg,
        noise_arg,
        bringup,
        *control_nodes,
        *perception_nodes,
        *planning_nodes,
        *app_nodes,
    ])
