#!/usr/bin/python3
"""
This module launches the mission
"""
# mypy: disable-error-code="arg-type"
import os
import json
from typing import List, Dict

import rospy
from eufs_msgs.msg import CanState
from asurt_msgs.msg import NodeStatus

from .module import Module
from .visualizer import Visualizer


class MissionLauncher:  # pylint: disable = too-many-instance-attributes
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

    def __init__(self, vizTopic: str, btnTopic: str) -> None:
        self.vizTopic = vizTopic
        self.btnTopic = btnTopic
        self.viz = Visualizer(vizTopic, btnTopic)
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

    def launch(self, mission: int, isBagSystem: bool) -> None:
        """
        Launches a mission based on the mission type received
        """
        if self.isLaunched:
            rospy.loginfo("Trying to launch a mission while another mission is running, ignoring")
            return

        if mission == CanState.AMI_DDT_INSPECTION_A:
            config = self.openConfig("staticA.json")
            self.missionType = "static A"
        elif mission == CanState.AMI_DDT_INSPECTION_B:
            config = self.openConfig("staticB.json")
            self.missionType = "static B"
        elif mission == CanState.AMI_TRACK_DRIVE:
            config = self.openConfig("Trackdrive.json")
            self.missionType = "Trackdrive"
        elif mission == -1:  # placeholder for testing:
            config = self.openConfig("testConfig.json")
            self.missionType = "test mission"
        else:
            raise ValueError(f"Invalid mission type: {mission}")

        for i in config["modules"]:
            if isBagSystem:
                if i["is_bag_pkg"]:
                    self.modules.append(
                        Module(i["pkg"], i["launch_file"], i["heartbeats_topic"], i["is_node_msg"])
                    )
            else:
                self.modules.append(
                    Module(i["pkg"], i["launch_file"], i["heartbeats_topic"], i["is_node_msg"])
                )

        for idx, module in enumerate(self.modules):
            module.launch()
            self.viz.addButton(7.5, -0.7 * idx, module)
            self.isLaunched = True

    def shutdown(self) -> None:
        """
        shutdown all modules and reset the visualizer
        """
        for module in self.modules:
            module.shutdown()

        # Recreate the visualizer to delete all buttons and reset marker counts
        del self.viz
        self.viz = Visualizer(self.vizTopic, self.btnTopic)

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

        if not self.isLaunched:
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

        self.viz.visualizeText(toVizualize)
