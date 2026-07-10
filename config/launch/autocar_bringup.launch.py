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
        Node(
            package='autocar_application', executable='mqtt_command_bridge',
            name='mqtt_command_bridge',
            parameters=[os.path.join(_config_dir, 'mqtt.yaml')],
        ),
    ])
