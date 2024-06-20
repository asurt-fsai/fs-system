import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the YAML file containing the parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    param_dir = os.path.join(get_package_share_directory('supervisor'), 'config', 'autoDemo_params.yaml')

    auto_demo_node = launch_ros.actions.Node(
        package='supervisor',
        executable='autoDemo',
        parameters=[param_dir],
        output='screen'
    )

    tkinter_node = launch_ros.actions.Node(
        package='supervisor',
        executable='interface',
        name='tkinter_node',
        namespace='tkinter_node',
        output='screen'
    )

    return launch.LaunchDescription([
        auto_demo_node,
        tkinter_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
