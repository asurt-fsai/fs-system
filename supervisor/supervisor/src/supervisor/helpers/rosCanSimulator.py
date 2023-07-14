#!/usr/bin/python3
"""
.

"""

from typing import Any

from enum import Enum

import rospy
from std_msgs.msg import Float32

from std_msgs.msg import Bool
from eufs_msgs.msg import CanState
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_srvs.srv import Trigger, TriggerResponse


class VCUState(Enum):
    """
    VCU state enumeration.
    """

    AS_OFF = 0
    AS_READY = 1
    AS_DRIVING = 2
    AS_FINISHED = 3
    AS_EBS = 4


class RosCanSimulator:  # pylint: disable=too-many-instance-attributes
    """
    ROS CAN simulator class.
    """

    def __init__(
        self,
        missionSelected: int,
        vcuVelTopic: str,
        vcuSteerTopic: str,
        rosCanStateTopic: str,
        rosCanVelTopic: str,
    ) -> None:
        self.missionSelected = missionSelected
        self.drivingFlag = False
        self.missionFlag = False
        self.cmdVel = 0.0
        self.cmdSteer = 0.0
        self.currentVel = 0.0

        self.canStatePub = rospy.Publisher(rosCanStateTopic, CanState, queue_size=10)
        self.currentVelPub = rospy.Publisher(
            rosCanVelTopic, TwistWithCovarianceStamped, queue_size=10
        )
        self.velPub = rospy.Publisher(vcuVelTopic, Float32, queue_size=10)
        self.steerPub = rospy.Publisher(vcuSteerTopic, Float32, queue_size=10)

        self.curState = VCUState.AS_OFF
        rospy.Service("vcu_next_state", Trigger, self.nextState)
        rospy.Service("vcu_prev_state", Trigger, self.prevState)
        rospy.Service("/ros_can/ebs", Trigger, self.ebs)

    def nextState(self, _: Any) -> str:
        """
        Next state
        """
        self.curState = VCUState(min(self.curState.value + 1, 4))
        response = TriggerResponse()
        response.success = True
        response.message = "Success"
        return response  # type: ignore

    def prevState(self, _: Any) -> str:
        """
        Previous state
        """
        self.curState = VCUState(max(self.curState.value - 1, 0))
        response = TriggerResponse()
        response.success = True
        response.message = "Success"
        return response  # type: ignore

    def ebs(self, _: Any) -> str:
        """
        Emergency brake service
        """
        self.curState = VCUState.AS_EBS
        response = TriggerResponse()
        response.success = True
        response.message = "Success"
        return response  # type: ignore

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data

    def missionFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the mission flag
        """
        self.missionFlag = msg.data

    def cmdCallback(self, msg: AckermannDriveStamped) -> None:
        """
        Callback for the drive command
        """
        if self.curState == VCUState.AS_DRIVING:
            self.cmdVel = msg.drive.speed
            self.cmdSteer = msg.drive.steering_angle
        else:
            self.cmdVel = 0
            self.cmdSteer = 0

    def currentVelCallback(self, msg: Float32) -> None:
        """
        Callback for the current velocity
        """
        self.currentVel = msg.data

    def run(self) -> None:
        """
        Publish velocity and state
        """
        velMsg = TwistWithCovarianceStamped()
        velMsg.twist.twist.linear.x = self.currentVel
        velMsg.header.stamp = rospy.Time.now()
        self.currentVelPub.publish(velMsg)

        canState = CanState()
        canState.as_state = self.curState.value
        canState.ami_state = self.missionSelected
        self.canStatePub.publish(canState)

        self.velPub.publish(self.cmdVel)
        self.steerPub.publish(self.cmdSteer)
