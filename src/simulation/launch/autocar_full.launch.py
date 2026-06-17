"""Launch the complete Autocar system in simulation mode.

Brings up:
- Gazebo Ignition with the Autocar world and robot model
- Simulation bridge for ROS-Gazebo communication
- All perception, planning, control, and application nodes
- RViz2 for visualization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for full Autocar simulation."""
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    headless = LaunchConfiguration("headless", default="false")
    world_name = LaunchConfiguration("world_name", default="autocar_garage")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo Ignition in headless mode",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="autocar_garage",
            description="World file name (without .sdf extension)",
        ),
    ]

    # Include simulation bringup
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("autocar_simulation"),
                "launch",
                "sim_bringup.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "headless": headless,
        }.items(),
    )

    # TODO: Include perception layer when implemented
    # perception_launch = IncludeLaunchDescription(...)

    # TODO: Include planning layer when implemented
    # planning_launch = IncludeLaunchDescription(...)

    # TODO: Include control layer when implemented
    # control_launch = IncludeLaunchDescription(...)

    # TODO: Include application layer when implemented
    # application_launch = IncludeLaunchDescription(...)

    return LaunchDescription(
        declared_arguments
        + [
            sim_bringup,
        ]
    )
