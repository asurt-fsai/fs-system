# pylint: disable-all
# mypy: ignore-errors
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # Configure environment
    base_path = os.path.realpath(
        get_package_share_directory("simple_pure_pursuit"),"config","simplepurepuirsuit_parameters.yaml"
    )  # also tried without realpath
    return LaunchDescription(
        [
            Node(
                package="simple_pure_pursuit",
                namespace="",
                executable="simple_pp_node",
                name="main",
                # parameters=[base_path+'/config/params.yaml']
            ),
           
        ]
    )
