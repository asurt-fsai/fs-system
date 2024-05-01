#!/usr/bin/python3
"""
.

"""

from typing import Any

from enum import Enum
from std_msgs.msg import Float32
import rclpy
from rclpy import Node
import time
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


class RosCanSimulator(Node):  # pylint: disable=too-many-instance-attributes
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

        self.canStatePub = self.create_publisher(CanState, rosCanStateTopic, 10)
        self.currentVelPub = self.create_publisher(TwistWithCovarianceStamped, rosCanVelTopic, 10)
        self.velPub = self.create_publisher(Float32, vcuVelTopic, 10)
        self.steerPub = self.create_publisher(Float32, vcuSteerTopic, 10)


        self.curState = VCUState.AS_OFF

        self.next_state_service = self.create_service(Trigger, 'vcu_next_state', self.nextState)
        self.prev_state_service = self.create_service(Trigger, 'vcu_prev_state', self.prevState)
        self.ebs_service = self.create_service(Trigger, '/ros_can/ebs', self.ebs)


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
        response = Trigger.Response()
        response.success = True
        response.message = "Success"
        return response  # type: ignore

    def ebs(self, _: Any) -> str:
        """
        Emergency brake service
        """
        self.curState = VCUState.AS_EBS
        response = Trigger.Response()
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
        velMsg.header.stamp = time.time()
        self.currentVelPub.publish(velMsg)

        canState = CanState()
        canState.as_state = self.curState.value
        canState.ami_state = self.missionSelected
        self.canStatePub.publish(canState)

        self.velPub.publish(self.cmdVel)
        self.steerPub.publish(self.cmdSteer)
