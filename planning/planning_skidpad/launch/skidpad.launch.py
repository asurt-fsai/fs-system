"""Module providing a function printing python version."""
import os
from launch import LaunchDescription  # type: ignore[attr-defined]
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:  # pylint: disable=invalid-name
    """Launch the skidpad path planner node."""
    config = os.path.join(get_package_share_directory("planning_skidpad"), "config", "params.yaml")
    skidpadPathPlannerNode = Node(
        package="planning_skidpad",
        executable="SkidPadPathPlannerNode",
        name="skidPadPathPlannerNode",
        parameters=[config],
    )
    launchDescription = LaunchDescription()
    launchDescription.add_action(skidpadPathPlannerNode)
    return launchDescription
