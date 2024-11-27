# pylint: disable-all
# mypy: ignore-errors
from logging import config
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser

def generate_launch_description():
    
    # Configure environment

    # Simulated time
    use_sim_time = LaunchConfiguration("use_sime_time", default="false")

    config = os.path.join(get_package_share_directory("planning_centerline_calc"), "config", "planning_parameters.yaml")

    planning_node = Node(
        package="planning_centerline_calc",
        executable="planning_centerline_calc_node",
        output="screen",
        parameters=[config],
    )
    transform_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_to_inverted',
        arguments=['0', '0', '0', '0', '-3.14159', '0','zed_left','zed_inverted'],
    )

    ld = LaunchDescription()

    # Add node 
    ld.add_action(planning_node)
    ld.add_action(transform_velodyne)

    return ld