"""
Supervisor main module
"""
from typing import Optional
from enum import Enum

import rospy

from std_msgs.msg import Bool, Float32
from eufs_msgs.msg import CanState
from geometry_msgs.msg import TwistWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from .helpers import MissionLauncher  # type: ignore
from .helpers import Visualizer  # type: ignore


class SuperState(Enum):
    """
    Enum class for the supervisor's state
    """

    WAITING = 0
    LAUNCHING = 1
    READY = 2
    RUNNING = 3
    STOPPING = 4
    FINISHED = 5


class Supervisor:  # pylint: disable=too-many-instance-attributes
    """
    This class is the supervisor's main class
    """

    def __init__(
        self,
        rosCanCmdTopic: str,
        drivingFlagTopic: str,
        missionFlagTopic: str,
        markersTopic: Optional[str] = None,
        btnTopic: Optional[str] = None,
    ) -> None:
        self.asState = CanState.AS_OFF
        self.amiState = CanState.AMI_NOT_SELECTED
        self.isFinished = False
        self.superState = SuperState.WAITING
        self.maxStopVelTh = 0.1
        self.currentVel = 0
        self.vel = 0
        self.steer = 0

        self.drivingFlagPub = rospy.Publisher(drivingFlagTopic, Bool, queue_size=10)
        self.missionFlagPub = rospy.Publisher(missionFlagTopic, Bool, queue_size=10)
        self.cmd = rospy.Publisher(rosCanCmdTopic, AckermannDriveStamped, queue_size=10)

        if markersTopic is not None and btnTopic is not None:
            self.visualizer = Visualizer(markersTopic, btnTopic)
            self.launcher = MissionLauncher(self.visualizer)
        else:
            self.launcher = MissionLauncher()

    def run(self) -> None:
        """
        Do the state machine transitions and actions
        """

        # Update launcher
        if self.superState != SuperState.WAITING:
            self.launcher.update()

        # Do transitions
        if self.superState == SuperState.WAITING:
            if self.amiState != CanState.AMI_NOT_SELECTED:
                self.superState = SuperState.LAUNCHING
                self.launcher.launch(self.amiState)
        elif self.superState == SuperState.LAUNCHING:
            if self.launcher.isReady():
                self.superState = SuperState.READY
        elif self.superState == SuperState.READY:
            if self.asState == CanState.AS_DRIVING:
                self.superState = SuperState.RUNNING
        elif self.superState == SuperState.RUNNING:
            if self.isFinished:
                self.superState = SuperState.STOPPING
        elif self.superState == SuperState.STOPPING:
            if self.currentVel < self.maxStopVelTh:
                self.superState = SuperState.FINISHED

        # Publish
        self.publishRosCanMessages()

    def publishRosCanMessages(self) -> None:
        """
        Publishes the command to the car
        """

        if self.superState in (SuperState.WAITING, SuperState.LAUNCHING, SuperState.READY):
            self.missionFlagPub.publish(False)
            self.drivingFlagPub.publish(False)
        elif self.superState in (SuperState.RUNNING, SuperState.STOPPING):
            self.missionFlagPub.publish(False)
            self.drivingFlagPub.publish(True)
        elif self.superState == SuperState.FINISHED:
            self.missionFlagPub.publish(True)
            self.drivingFlagPub.publish(False)

        self.cmd.publish(self.getCmdMessage())

    def getCmdMessage(self) -> AckermannDriveStamped:
        """
        Publishes the command to the car
        after changing the steer and vel
        msg to AckermannDriveStamped
        """
        cmdMsg = AckermannDriveStamped()
        if self.superState in (
            SuperState.WAITING,
            SuperState.LAUNCHING,
            SuperState.READY,
            SuperState.FINISHED,
        ):
            cmdMsg.drive.speed = 0
            cmdMsg.drive.steering_angle = 0
        elif self.superState == SuperState.RUNNING:
            cmdMsg.drive.speed = self.vel
            cmdMsg.drive.steering_angle = self.steer
        elif self.superState == SuperState.STOPPING:
            cmdMsg.drive.steering_angle = self.steer
            if self.currentVel > 0.1:
                targetVel = 0.5 * self.currentVel
            else:
                targetVel = 0
            cmdMsg.drive.speed = targetVel
        cmdMsg.header.stamp = rospy.Time.now()
        return cmdMsg

    def canStateCallback(self, msg: CanState) -> None:
        """
        Callback to retreive the VCU's state and selected mission
        """
        self.asState = msg.as_state
        self.amiState = msg.ami_state

    def isFinishedCallback(self, msg: Bool) -> None:
        """
        Callback for the mission's finished flag
        """
        self.isFinished = msg.data

    def velCallback(self, msg: Float32) -> None:
        """
        Callback function for the velocity
        """
        self.vel = msg.data

    def steerCallback(self, msg: Float32) -> None:
        """
        Callback function for the steering angle
        """
        self.steer = msg.data

    def currentVelCallback(self, msg: TwistWithCovarianceStamped) -> None:
        """
        Callback for the current velocity
        """
        self.currentVel = msg.twist.twist.linear.x
