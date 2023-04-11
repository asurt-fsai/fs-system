#!/usr/bin/python3
"""
.

"""
import os
import json
from typing import List, Dict, Optional
import rospy
from std_msgs.msg import Bool

from supervisor.supervisor_functions.launcher import Launcher


class MissionLauncher:  # pylint: disable = too-many-instance-attributes
    """
    This class launches the mission
    based on the mission type received
    """

    def __init__(self) -> None:
        self.launcher: Optional[Launcher] = None
        self.isLaunched = False
        self.gotGoSignal = False
        self.sentGoSignal = False
        self.goPub = rospy.Publisher("/go_signal", Bool, queue_size=10, latch=True)
        self.missionFlagPub = rospy.Publisher("/mission_flag", Bool, queue_size=10, latch=True)
        self.drivingFlagPub = rospy.Publisher("/driving_flag", Bool, queue_size=10, latch=True)
        self.drivingFlag = False
        self.missionFlag = False
        self.vel = 0.0
        self.steer = 0.0

    def goCallback(self, msg: Bool) -> None:
        """
        Callback function for the go signal
        """
        if msg.data:
            self.gotGoSignal = True

    def openConfig(self, fileName: str) -> Dict[str, List[Dict[str, str]]]:
        """
        Open the config file and return it as a dict.
        """
        with open(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../nodes/", fileName),
            encoding="utf-8",
        ) as configFile:
            config = json.load(configFile)

        return config  # type: ignore

    def missionCallback(self, msg: Bool) -> None:
        """
        Callback function for the mission type
        """
        if msg.data == 0:
            config = self.openConfig("staticA.json")
            self.launcher = Launcher(config, "/marker/supervisor", "supervisor_markers")
        if msg.data == 1:
            config = self.openConfig("staticB.json")
            self.launcher = Launcher(config, "/marker/supervisor", "supervisor_markers")

    def setVel(self, vel: float) -> None:
        """
        Sets the velocity
        """
        self.vel = vel

    def setSteer(self, steer: float) -> None:
        """
        Sets the steering angle
        """
        self.steer = steer

    def launch(self) -> None:
        """
        Launches the mission
        """

        if not self.launcher:
            raise ValueError("Launcher is not initialized.")

        if self.launcher is not None and not self.isLaunched:
            self.launcher.launch()
            self.isLaunched = True

        if self.isLaunched:
            extraText = [
                ["Mission Flag", "Completed" if self.missionFlag else "Not Completed", 2],
                ["Driving Flag", str(self.drivingFlag), 2],
                ["Go Signal", str(self.gotGoSignal), 2],
                ["Sent Go Signal", str(self.sentGoSignal), 2],
                ["Steering Angle", str(self.steer), 2],
                ["Velocity", str(self.vel), 2],
            ]
            self.launcher.update(extraText)  # type: ignore

        if self.isLaunched and self.gotGoSignal and not self.sentGoSignal:
            self.sentGoSignal = True
            self.goPub.publish(True)

    def publishFlags(self) -> None:
        """
        Publishes the driving and mission flag
        """
        self.drivingFlagPub.publish(self.drivingFlag)
        self.missionFlagPub.publish(self.missionFlag)

    def stateCallback(self, msg: Bool) -> None:
        """
        Callback function for the state
        """
        if msg.data == 0:
            self.drivingFlag = False
            self.missionFlag = False
        if msg.data == 1:
            self.drivingFlag = True
            self.missionFlag = False
        if msg.data == 2:
            self.drivingFlag = False
            self.missionFlag = True

        self.publishFlags()
