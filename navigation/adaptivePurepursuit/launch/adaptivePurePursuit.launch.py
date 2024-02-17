import os
import launch
from launch import actions
from launch_ros.actions import Node

def generate_launch_description():
    adaptive_pp_node = Node(
        package='adaptivePurepursuit_py',
        executable='adaptivePurepursuit',
        name='adaptivePurepursuit',
        parameters= [
            {"/gains/lookahead_distance": 4.2},
            {"/look_ahead_constant": 2.0},
            {"/time_step": 0.1},
            {"/speed/max": 3.5},
            {"/speed/min": 0.5},
            {"/gains/proportional": 1.5},
            {"/gains/integral": 1},
            {"/gains/differential": 0.5},
            {"/gain": 0.3}
        ]
    )

    return launch.LaunchDescription([
        adaptive_pp_node
    ])

if __name__ == '__main__':
    generate_launch_description()