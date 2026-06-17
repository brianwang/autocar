"""Launch Gazebo Ignition with autocar_car robot model and simulation bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindPackageShare
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

    # Build world file path from package share
    world_file = PathJoinSubstitution([
        FindPackageShare(sim_pkg), 'worlds',
        LaunchConfiguration('world')
    ]) + '.sdf'

    # Start Gazebo server
    gz_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', '--world-name', world_name, world_file],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': os.path.join(sim_dir, 'models')},
        condition=LaunchConfiguration('headless') == 'true',
    )

    # Start Gazebo with GUI (non-headless)
    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '--world-name', world_name, world_file],
        output='screen',
        additional_env={'IGN_GAZEBO_RESOURCE_PATH': os.path.join(sim_dir, 'models')},
        condition=LaunchConfiguration('headless') == 'false',
    )

    # Spawn robot via ign service
    robot_sdf = os.path.join(sim_dir, 'models', 'autocar_car', 'model.sdf')
    spawn_robot = ExecuteProcess(
        cmd=[
            'ign', 'service', '-s',
            '/world/{}/create'.format('test_obstacle'),
            '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean',
            '--timeout', '10',
            '--req', 'sdf_filename: "{}"'.format(robot_sdf),
        ],
        output='screen',
    )

    # Start simulation bridge node
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
        name='map_to_odom_static',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    return LaunchDescription([
        world_arg,
        noise_arg,
        headless_arg,
        gz_server,
        gz_gui,
        spawn_robot,
        bridge_node,
        tf_static_node,
    ])
