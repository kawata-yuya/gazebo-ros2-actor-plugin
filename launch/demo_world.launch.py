import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    package_name = "gazebo_ros2_actor_plugin"
    launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_name = LaunchConfiguration('world_name', default='demo.world')

    ld = LaunchDescription()

    world = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'worlds',
        world_name,
    ])

    declare_world_name_cmd = DeclareLaunchArgument(
        'world_name', default_value='demo.world',
        description='Launch world name')

    # Launch Gazebo
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    # Add the commands to the launch description
    ld.add_action(declare_world_name_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld
