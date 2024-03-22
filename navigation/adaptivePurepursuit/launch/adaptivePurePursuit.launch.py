import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    base_path = os.path.realpath(get_package_share_directory('kinematic_bicycle')) # also tried without realpath
    rviz_path=base_path+'/bicycle.rviz'
    return LaunchDescription([
        Node(
            package='kinematic_bicycle',
            namespace='car',
            executable='kinematic_bicycle',
            name='main'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            output='screen', 
            arguments=['-d'+str(rviz_path)]
        ),
        Node(
            package='kinematic_bicycle',
            namespace='',
            executable='path_gen',
            name='main'
        ),
        Node(
        package='adaptivePurepursuit',
        executable='adaptive_pp_node',
        name='main',
        parameters= [
            {"/gains/lookahead_distance": 4.2},
            {"/look_ahead_constant": 2.0},
            {"/time_step": 0.1},
            {"/speed/max": 3.5},
            {"/speed/min": 0.5},
            {"/gains/proportional": 1.5},
            {"/gains/integral": 1},
            {"/gains/differential": 0.5},
            {"/gain": 0.3},
            {"/speed_constant": 6.5}
        ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()