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
