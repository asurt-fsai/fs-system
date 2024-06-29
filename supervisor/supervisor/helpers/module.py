"""
Module class to launch and shutdown modules (launch files)
"""
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
import time
from rclpy.node import Node
import launch
from asurt_msgs.msg import NodeStatus
from .intervalTimer import IntervalTimer
import launch.actions
import launch.substitutions
import launch_ros.actions
import os


class Module(Node):  # pylint: disable=too-many-instance-attributes
    """
    Launches a module (launch file) and checks if it is alive using the optional heartbeat topic
    It can also shutdown and restart the module

    Parameters
    ----------
    pkg : str
        The name of the package.
    launch : str
        The name of the launch file.
    heartbeat : str, optional
        Checks if the node is alive.
    isHeartbeatNodestatus : bool, optional
        If the heartbeat topic is a NodeStatus message, by default True
        If false, the topic is assumed to be the same type as the topic the module publishes
    """

    def __init__(
        self,
        pkg: str,
        launchFile: str,
        heartbeat: Optional[str] = None,
        isHeartbeatNodestatus: bool = True,
    ) -> None:
        
        super().__init__("Module")
        
        self.pkg = pkg
        self.launchFile = launchFile
        self.state = NodeStatus.SHUTDOWN
        self.moduleHandle = None
        self.scheduleRestart = False
        self.hasHeartbeat = heartbeat is not None
        self.rate = 0.0

        if self.hasHeartbeat:
            if isHeartbeatNodestatus:
                self.create_subscription(NodeStatus, heartbeat,  self.heartbeatCallback, 10)

            else:
                self.create_subscription(NodeStatus, heartbeat,  self.msgCallback, 10)

                #rospy.Subscriber(heartbeat, rospy.AnyMsg, self.msgCallback)

            self.heartbeartRateThread = IntervalTimer(1, self.updateHeartbeatRate)
            self.lastHeartbeatTime = time.time()
            self.heartbeatCount = 0.0

        # Parameter Validation
        try:
            assert self.pkg is not None and self.launchFile is not None
        except Exception as exc:
            errMsg = "Supervisor.Module: must have a pkg and launch file and not None"
            raise TypeError(errMsg) from exc

    def __repr__(self) -> str:
        return f"{self.pkg} {self.launchFile}"

    def restart(self) -> None:
        """
        Restarts the module.
        """
        self.shutdown()
        self.launch()

    def launch(self) -> None:
        """
        Launches the module.
        """

        '''
        self.state = NodeStatus.STARTING
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        roslaunchFile = roslaunch.rlutil.resolve_launch_arguments([self.pkg, self.launchFile])

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunchFile)
        parent.start()
        '''
        '''
        use_sim_time = LaunchConfiguration("use_sim_time", default="false")
            param_dir = os.path.join(get_package_share_directory('supervisor'), 'config', 'staticA_params.yaml')

            static_a_node = launch_ros.actions.Node(
                package='supervisor',
                executable='staticAtest',
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
                DeclareLaunchArgument(
                    'param_dir',
                    default_value=param_dir,
                    description='Full path to the parameter file to load'),
                static_a_node,
                tkinter_node
            ])

        '''

        # # Create launch description
        

        # # Set node status to STARTING
        # self.state =NodeStatus.STARTING

        # # Resolve launch file arguments
        # launch_file_path = self.launchFile
        # # launch_file_path = launch.substitutions.LaunchConfiguration('launch_file')
        # pkg_path = self.pkg  #launch.substitutions.LaunchConfiguration('pkg')
        # resolved_launch_file = launch.actions.IncludeLaunchDescription(
        #     launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path),
        #     launch_arguments={'pkg': pkg_path}.items())

        # # Add actions to launch description
        # ld.add_action(resolved_launch_file)

        # # Execute launch description
        # return launch.LaunchDescription([ld])
        # Define the path to your launch file
         # Create the launch service
        
        
        launch_service = launch.LaunchService()
    # Define the path to your launch file
        launch_file_path = os.path.join(get_package_share_directory('supervisor'), 'launch', self.launchFile)

        # Include the launch description from your staticA_launch.py file
        launch_description = launch.LaunchDescription([
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file_path)
            ),
        ])
    

        # Include the launch description in the launch service
        launch_service.include_launch_description(launch_description)

        # Run the launch service
        launch_service.run()



    def shutdown(self) -> None:
        """
        Shuts down the module.
        """
        if self.moduleHandle is not None:
           if self.hasHeartbeat:
                self.heartbeartRateThread.stop()
                self.heartbeartRateThread = IntervalTimer(1, self.updateHeartbeatRate)
            
        self.state = NodeStatus.SHUTDOWN

    def heartbeatCallback(self, msg: NodeStatus) -> None:
        """
        Callback for the heartbeat topic

        Parameters
        ----------
        msg : NodeStatus
            The message from the heartbeat topic
        """
        self.state = msg.status

        # increment the count of messages since last update of the heartbeat rate
        self.heartbeatCount += 1.0

    def msgCallback(self, _: NodeStatus) -> None:
        """
        Callback for the topic the module publishes (used if no heartbeat topic is available)

        Parameters
        ----------
        msg: Any
            The message from the topic
        """
        self.state = NodeStatus.RUNNING
        # increment the count of messages since last update of the heartbeat rate
        self.heartbeatCount += 1.0

    def updateHeartbeatRate(self) -> None:
        """
        Updates the heartbeat rate, called using a looping thread
        """
        timeDiff = time.time() - self.lastHeartbeatTime
        self.lastHeartbeatTime = time.time
        newHeartbeartRate = self.heartbeatCount / (timeDiff + 0.001)  # avoid division by
        self.heartbeatCount = 0

        beta = 0.3  # smoothing factor
        self.rate = beta * self.rate + (1.0 - beta) * newHeartbeartRate
        if newHeartbeartRate < 1:
            self.state = NodeStatus.UNRESPONSIVE

    def __del__(self) -> None:
        self.shutdown()

        
