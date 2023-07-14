#!/usr/bin/python3
"""
Autonomous Demo
"""
import time

import rospy

from std_msgs.msg import Float32, Bool, Int16
from std_srvs.srv import Trigger, TriggerRequest


class AutonomousDemo:  # pylint: disable=too-many-instance-attributes
    """
    AutonomousDemo
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(
        self, velTopic: str, steerTopic: str, ebsTopic: str, isFinishedTopic: str, stateTopic: str
    ) -> None:
        rospy.loginfo("Starting Autonomous Demo")

        self.drivingFlag = False
        self.started = False

        self.currentVel = 0.0
        self.timePrevStep = time.time()
        self.dist = 0.0
        self.drevT = 0.0

        vcuCurrVelTopic = rospy.get_param("/vcu/curr_vel")
        rospy.Subscriber(vcuCurrVelTopic, Float32, self.currentVelCallback)

        rospy.wait_for_service(ebsTopic)
        self.ebsService = rospy.ServiceProxy(ebsTopic, Trigger)

        self.velPub = rospy.Publisher(velTopic, Float32, queue_size=10)
        self.steerPub = rospy.Publisher(steerTopic, Float32, queue_size=10)
        self.statePub = rospy.Publisher(stateTopic, Int16, queue_size=10)
        self.finishPub = rospy.Publisher(isFinishedTopic, Bool, queue_size=10, latch=True)

        self.maxSteer = float(rospy.get_param("/maxSteer"))

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True

    def currentVelCallback(self, msg: Float32) -> None:
        """
        Callback function
        """
        timeCurrentStep = time.time()
        self.drevT = timeCurrentStep - self.timePrevStep
        self.timePrevStep = timeCurrentStep
        self.currentVel = msg.data

    def run(self) -> None:
        """
        Run Autonomous Demo
        """

        if not self.started:
            rospy.loginfo("Waiting for driving flag")
            return

        rospy.sleep(2)
        self.statePub.publish(1)  # 1 is for driving
        self.steerPub.publish(-self.maxSteer)
        rospy.sleep(10)
        self.steerPub.publish(self.maxSteer)
        rospy.sleep(10)
        self.steerPub.publish(0)
        rospy.sleep(10)

        while self.dist < 10:
            self.dist += self.currentVel * self.drevT
            self.velPub.publish(1.0)

        rospy.sleep(5)
        self.velPub.publish(0.0)
        rospy.sleep(5)

        self.dist = 0.0
        while self.dist < 10:
            self.dist += self.currentVel * self.drevT
            self.velPub.publish(1.0)

        ebs = TriggerRequest()
        self.ebsService(ebs)

        self.statePub.publish(2)  # 2 is for finished
        self.finishPub.publish(True)  # 1 is for finished
