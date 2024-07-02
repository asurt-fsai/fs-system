# pylint: disable-all
"""

This is the launch file for the adaptivePurePursuit package. It defines a function
generateLaunchdescription() that generates a launch description that launches
the adaptive_pp_node with the parameters specified in the adaptive_pp_parameters.yaml file.

"""

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription  # type: ignore[attr-defined]
from launch_ros.actions import Node


def generateLaunchdescription() -> LaunchDescription:
    """
    This function generates a launch description that launches the adaptive_pp_node
    with the parameters specified in the adaptive_pp_parameters.yaml file.

    returns:
        LaunchDescription: The launch description with the adaptive_pp_node.
    """
    config = os.path.join(
        get_package_share_directory("adaptivePurepursuit"), "config", "adaptive_pp_parameters.yaml"
    )

    adaptive_pp_node = Node(
        package="adaptivePurepursuit",
        executable="adaptive_pp_node",
        output="screen",
        parameters=[config],
    )

    ld = LaunchDescription()
    ld.add_action(adaptive_pp_node)
    return ld


if __name__ == "__main__":
    generateLaunchdescription()
