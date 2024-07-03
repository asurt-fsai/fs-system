"""
Module class to launch and shutdown modules (launch files)
"""
from typing import Optional
import subprocess
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
import subprocess
import signal
from ament_index_python.packages import get_package_share_directory


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
        self.isHeartbeatNodestatus = isHeartbeatNodestatus
        self.rate = 0.0

        terminal_processes = {}
        

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

    def run_terminal_command(command):
        try:
        # Open a terminal and execute the command
            subprocess.run(['gnome-terminal', '--', 'bash', '-c', command])
        except FileNotFoundError:
            print("Error: gnome-terminal is not installed or not found on your system.")

    def launch(self,pkgs, launch_files) -> None:
        """
        Launches the module.
        """
        for idx, (launch_file, pkg) in enumerate(zip(launch_files, pkgs), start=1):
        # Construct the command to source ROS 2 environment and launch the file
            command = f'source /opt/ros/humble/setup.bash && ros2 launch {pkg} {launch_file}'
            pid = self.run_terminal_command(command)
            if pid is not None:
                self.terminal_processes[idx] = pid  # Store the process ID with its associated ID
                print(f"Launched terminal {idx} for {pkg} - {launch_file}")
            else:
                print(f"Failed to launch terminal for {pkg} - {launch_file}")

    def shutdown(self) -> None:
        """
        Shuts down the module.
        """
        if self.moduleHandle is not None:
           if self.hasHeartbeat:
                self.heartbeartRateThread.stop()
                self.heartbeartRateThread = IntervalTimer(1, self.updateHeartbeatRate)
            
        self.state = NodeStatus.SHUTDOWN

    def close_terminal(self,terminal_id):
        if terminal_id in self.terminal_processes:
            pid = self.terminal_processes[terminal_id]
            try:
                os.kill(pid, signal.SIGINT)  # Send SIGINT signal to terminate the process
                del self.terminal_processes[terminal_id]  # Remove from dictionary after termination
                print(f"Closed terminal {terminal_id}")
            except ProcessLookupError:
                print(f"Failed to close terminal {terminal_id}: Process not found")
        else:
            print(f"Terminal with ID {terminal_id} not found.")

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

    def updateHeartbeatRate(self, StatusTopic) -> None:
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

    def run_terminal_command(command):
        try:
        # Open a terminal and execute the command
          subprocess.run(['gnome-terminal', '--', 'bash', '-c', command])
        except FileNotFoundError:
            print("Error: gnome-terminal is not installed or not found on your system.")

    def __del__(self) -> None:
        self.shutdown()

        
