"""Launch the Gazebo Ignition simulation environment for Autocar."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Autocar simulation."""
    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_file = LaunchConfiguration("world_file", default="")
    headless = LaunchConfiguration("headless", default="false")
    rviz_config = LaunchConfiguration("rviz_config", default="")
    enable_bridge = LaunchConfiguration("enable_bridge", default="true")

    # Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        DeclareLaunchArgument(
            "world_file",
            default_value="",
            description="Path to the SDF world file",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run Gazebo Ignition in headless mode",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="Path to RViz2 configuration file",
        ),
        DeclareLaunchArgument(
            "enable_bridge",
            default_value="true",
            description="Enable ROS-Gazebo bridge",
        ),
    ]

    # Gazebo Ignition server
    gz_server = ExecuteProcess(
        cmd=[
            FindExecutable(name="ign"),
            "gazebo",
            "-v", "4",
            "-r",
            world_file,
        ],
        condition=IfCondition(LaunchConfiguration("headless")),
        output="screen",
    )

    # Gazebo Ignition GUI
    gz_gui = ExecuteProcess(
        cmd=[
            FindExecutable(name="ign"),
            "gazebo",
            "-v", "4",
            "-g",
            world_file,
        ],
        condition=IfCondition(LaunchConfiguration("headless")),
        output="screen",
    )

    # Simulation bridge node
    simulation_bridge = Node(
        package="autocar_simulation",
        executable="simulation_bridge",
        name="simulation_bridge",
        output="screen",
        parameters=[PathJoinSubstitution(
            [FindPackageShare("autocar_simulation"), "config", "sim_params.yaml"]
        )],
        condition=IfCondition(enable_bridge),
    )

    # RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", rviz_config,
        ],
        condition=IfCondition(LaunchConfiguration("rviz_config")),
    )

    return LaunchDescription(
        declared_arguments
        + [
            gz_server,
            gz_gui,
            simulation_bridge,
            rviz2,
        ]
    )
