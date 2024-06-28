"""Launch file for the adaptivePurePursuit package."""

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription  # type: ignore[attr-defined]
from launch_ros.actions import Node


def generateLaunchdescription() -> LaunchDescription:
    """Generate launch description with multiple components"""
    config = os.path.join(
        get_package_share_directory("adaptivePurepursuit"), "config", "adaptive_pp_parameters.yaml"
    )
    basePath = os.path.realpath(
        get_package_share_directory("kinematic_bicycle")
    )  # also tried without realpath
    rvizPath = basePath + "/bicycle.rviz"
    launchDescription = LaunchDescription(
        [
            Node(
                package="kinematic_bicycle",
                namespace="car",
                executable="kinematic_bicycle",
                name="main",
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d" + str(rvizPath)],
            ),
            Node(package="kinematic_bicycle", namespace="", executable="path_gen", name="main"),
            Node(
                package="adaptivePurepursuit",
                executable="adaptive_pp_node",
                name="main",
                parameters=[
                    {"/look_ahead_constant": 2.0},
                    {"/time_step": 0.1},
                    {"/speed/maximum": 3.5},
                    {"/speed/min": 0.5},
                    {"/gains/proportional": 1.5},
                    {"/gains/integral": 1},
                    {"/gains/differential": 0.5},
                    {"/gain": 0.3},
                    {"/speed_constant": 6.5},
                ],
            ),
            Node(
                package="adaptivePurepursuit",
                executable="adaptive_pp_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
    return launchDescription


if __name__ == "__main__":
    generateLaunchdescription()
