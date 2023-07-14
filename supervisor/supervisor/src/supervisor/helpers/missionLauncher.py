"""
.
"""
import os
import json
from typing import List, Dict, Optional

import rospy
from eufs_msgs.msg import CanState
from asurt_msgs.msg import NodeStatus

from .module import Module
from .visualizer import Visualizer

AMIToConfig = {
    CanState.AMI_DDT_INSPECTION_A: "staticA",
    CanState.AMI_DDT_INSPECTION_B: "staticB",
    CanState.AMI_AUTONOMOUS_DEMO: "autonomousDemo",
    CanState.AMI_AUTOCROSS: "autocross",
    CanState.AMI_SKIDPAD: "skidpad",
    CanState.AMI_ACCELERATION: "acceleration",
    CanState.AMI_TRACK_DRIVE: "trackDrive",
}


class MissionLauncher:
    """
    This class launches the mission
    based on the mission type received


    Parameters
    ----------
    markersTopic : str
        The visualizer topic to publish the text markers to
    btnTopic : str
        The visualizer topic to publish the buttons to
    """

    def __init__(self, visualizer: Optional[Visualizer] = None) -> None:
        self.visualizer = visualizer
        self.missionType = "Not Selected"
        self.isLaunched = False
        self.modules: List[Module] = []

    def openConfig(self, fileName: str) -> Dict[str, List[Dict[str, str]]]:
        """
        Open the config file and return it as a dict.
        """
        with open(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../configs/", fileName),
            encoding="utf-8",
        ) as configFile:
            config = json.load(configFile)

        return config  # type: ignore

    def launch(self, mission: int) -> None:
        """
        Launches a mission based on the mission type received
        """

        if self.isLaunched:
            rospy.loginfo("Trying to launch a mission while another mission is running, ignoring")
            return

        if mission == -1:
            config = self.openConfig("testConfig.json")
            self.missionType = "test mission"
        else:
            try:
                config = self.openConfig(AMIToConfig[mission] + ".json")
                self.missionType = AMIToConfig[mission]
            except KeyError as exc:
                raise KeyError(f"Invalid mission type: {mission}") from exc

        for i in config["modules"]:
            self.modules.append(
                Module(i["pkg"], i["launch_file"], i["heartbeats_topic"], bool(i["is_node_msg"]))
            )

        for idx, module in enumerate(self.modules):
            module.launch()
            self.isLaunched = True

            if self.visualizer:
                self.visualizer.addButton(7.5, -0.7 * idx, module)

    def shutdown(self) -> None:
        """
        Shuts down all modules and resets the visualizer
        """

        for module in self.modules:
            module.shutdown()

        # Recreate the visualizer to delete all buttons and reset marker counts
        if self.visualizer:
            self.visualizer = self.visualizer.deleteAndReturnNewVisualizer()

        # Reset class variables
        self.missionType = "Not Selected"
        self.isLaunched = False
        self.modules = []

    def isReady(self) -> bool:
        """
        Returns True only if all modules have been launched and
        are running (publishing heartbeat state = RUNNING)
        Note: modules with no heartbeat topics are skipped (assumed to have launched)
        """
        for module in self.modules:
            if module.hasHeartbeat and module.state != NodeStatus.RUNNING:
                return False
        return True

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

        def getTextColor(nodeStatus: int) -> str:
            if nodeStatus in [NodeStatus.STARTING, NodeStatus.READY]:
                return "y"
            if nodeStatus in [NodeStatus.ERROR, NodeStatus.SHUTDOWN, NodeStatus.UNRESPONSIVE]:
                return "r"
            return "w"

        toVizualize = []
        for idx, module in enumerate(self.modules):
            color = getTextColor(module.state)
            toVizualize.append([0, -0.7 * idx, module.pkg, color])
            toVizualize.append([2.5, -0.7 * idx, states[module.state], color])
            toVizualize.append([5, -0.7 * idx, f"{module.rate:.2f} hz", color])

        nElements = len(toVizualize) // 3
        if extraText is not None:
            for idx, txt in enumerate(extraText):
                toVizualize.append([0, -0.7 * (idx + nElements), txt[0], txt[2]])
                toVizualize.append([4, -0.7 * (idx + nElements), txt[1], txt[2]])

        self.visualizer.visualizeText(toVizualize)
