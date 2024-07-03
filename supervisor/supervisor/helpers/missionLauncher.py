import os
import json
from typing import List, Dict, Optional
import asyncio
import rclpy
import time
from eufs_msgs.msg import CanState
from asurt_msgs.msg import NodeStatus
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from .module import Module
from .visualizer import Visualizer
import subprocess
import signal
from ament_index_python.packages import get_package_share_directory


AMIToConfig = {
    CanState.AMI_DDT_INSPECTION_A: "staticA",
    CanState.AMI_DDT_INSPECTION_B: "staticB",
    CanState.AMI_AUTONOMOUS_DEMO: "autonomousDemo",
    CanState.AMI_AUTOCROSS: "autocross",
    CanState.AMI_SKIDPAD: "skidpad",
    CanState.AMI_ACCELERATION: "acceleration",
    CanState.AMI_TRACK_DRIVE: "trackDrive",
}


class MissionLauncher(Node):
    def __init__(self, visualizer: Optional[Visualizer] = None) -> None:
        super().__init__('mission_launcher_node')
        self.visualizer = visualizer
        self.missionType = "Not Selected"
        self.isLaunched = False
        self.modules: List[Module] = []
        self.terminal_processes = {}

    def openConfig(self, fileName: str) -> Dict[str, List[Dict[str, str]]]:
        with open(
            os.path.join(get_package_share_directory('supervisor'), 'json', fileName),
            encoding="utf-8",
        ) as configFile:
            config = json.load(configFile)
        return config

    def launch(self, mission: int) -> None:
        '''
        Launches a mission based on the mission type
        '''
        if self.isLaunched:
            self.get_logger().info("Trying to launch a mission while another mission is running, ignoring")
            return
        else:
            self.get_logger().info("Launching mission")

        if mission == -1:
            config = self.openConfig("testConfig.json")
            self.missionType = "test mission"
        else:
            try:
                config = self.openConfig(AMIToConfig[mission] + ".json")
                self.missionType = AMIToConfig[mission]
                self.get_logger().info(f"Launching mission: {self.missionType}")
            except KeyError as exc:
                raise KeyError(f"Invalid mission type: {mission}") from exc

        for module_config in config["modules"]:
            module = Module(
                module_config["pkg"],
                module_config["launch_file"],
                module_config["heartbeats_topic"],
                bool(module_config["is_node_msg"])
            )
            self.modules.append(module)
            
            # Check if the module has a heartbeat topic and subscribe
            if module.hasHeartbeat:
                self.subscribe_to_heartbeat(module_config["heartbeats_topic"], module.isHeartbeatNodestatus)

        packages = [module.pkg for module in self.modules]
        launch_files = [module.launchFile for module in self.modules]
        
        self.launch_ros2_files(packages, launch_files)


    def shutdown(self) -> None:
        for module in self.modules:
            module.shutdown()

        self.missionType = "Not Selected"
        self.isLaunched = False
        self.modules = []

    def restart(self) -> None:
        for module in self.modules:
            self.close_terminal(1)
            self.launch_ros2_files([module.pkg], [module.launchFile])

    def isReady(self) -> bool:
        for module in self.modules:
            if module.hasHeartbeat and module.state != NodeStatus.RUNNING:
                return False
        return True
    

    def run_terminal_command(self,command):
        '''
        Run a command in a new terminal 
        '''
        try:
            # Open a terminal and execute the command
            process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', command])
            # Store the terminal's process ID
            return process.pid  # Return the process ID
        except FileNotFoundError:
            print("Error: gnome-terminal is not installed or not found on your system.")

    def launch_ros2_files(self,pkgs, launch_files):
        '''
        Launch ROS 2 launch files in separate terminals

        parameters
        ----------
        pkgs: List[str]
            List of package names
        launch_files: List[str]
            List of launch file names
        '''
        for idx, (launch_file, pkg) in enumerate(zip(launch_files, pkgs), start=1):
            # Construct the command to source ROS 2 environment and launch the file
            command = f'source /opt/ros/humble/setup.bash && ros2 launch {pkg} {launch_file}'
            pid = self.run_terminal_command(command)
            if pid is not None:
                self.terminal_processes[idx] = pid  # Store the process ID with its associated ID
                print(f"Launched terminal {idx} for {pkg} - {launch_file}")
            else:
                print(f"Failed to launch terminal for {pkg} - {launch_file}")

    def close_terminal(self,terminal_id):
        '''
        close a terminal with a given ID
        
        parameters
        ----------
        terminal_id: int
            The ID of the terminal to close
        '''
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

    def subscribe_to_heartbeat(self, heartbeat_topic: str, is_node_msg: bool) -> None:
        if is_node_msg:
            self.create_subscription(NodeStatus, heartbeat_topic, self.heartbeatCallback, 10)
        else:
            self.create_subscription(NodeStatus, heartbeat_topic, self.msgCallback, 10)

        self.heartbeartRateThread = self.create_timer(1.0, self.updateHeartbeatRate)
        self.lastHeartbeatTime = time.time()
        self.heartbeatCount = 0.0

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
            self.restart()


    def update(self) -> None:
        """
        Updates the visualizer for all modules along with adding extra information
        Current extra information:
        - Mission Type (static A, static B, etc)
        """

        if not self.isLaunched or not self.visualizer:
            return

        extraText = [
            ["Mission Type", self.missionType, 2],
        ]

        states = ["starting", "ready", "running", "error", "shutdown", "unreponsive"]
    
    def subscribe_to_heartbeat(self, heartbeat_topic: str, is_node_msg: bool) -> None:
        if is_node_msg:
            self.create_subscription(NodeStatus, heartbeat_topic, self.heartbeatCallback, 10)
        else:
            self.create_subscription(NodeStatus, heartbeat_topic, self.msgCallback, 10)

        self.heartbeartRateThread = self.create_timer(1.0, self.updateHeartbeatRate)
        self.lastHeartbeatTime = time.time()
        self.heartbeatCount = 0.0

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
            self.restart()