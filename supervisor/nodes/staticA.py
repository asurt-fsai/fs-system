#!/usr/bin/python3
"""
Static A
"""
import time

import rospy
from std_msgs.msg import Float32, Bool, Int16
from tf_helper.StatusPublisher import StatusPublisher
import numpy as np


class StaticA:
    """
    Static A
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(self) -> None:
        self.drivingFlag = False
        self.started = False

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True

    def run(self) -> None:
        """
        Run Static A
        """
        rospy.loginfo("Starting Static A")
        velTopic = rospy.get_param("/control/velocity")
        steerTopic = rospy.get_param("/control/steering")
        isFinishedTopic = rospy.get_param("/finisher/is_finished")
        stateTopic = rospy.get_param("/state")

        velPub = rospy.Publisher(velTopic, Float32, queue_size=10)
        steerPub = rospy.Publisher(steerTopic, Float32, queue_size=10)
        statePub = rospy.Publisher(stateTopic, Int16, queue_size=10)
        finishPub = rospy.Publisher(isFinishedTopic, Bool, queue_size=10, latch=True)
        maxSteer = float(rospy.get_param("/maxSteer"))

        rospy.sleep(2)
        statePub.publish(1)  # 1 is for driving
        steerPub.publish(-maxSteer)
        rospy.sleep(10)
        steerPub.publish(maxSteer)
        rospy.sleep(10)
        steerPub.publish(0)
        rospy.sleep(10)

        timeStart = time.time()
        while time.time() - timeStart < 10:
            vel = 2 * np.pi * 200 * 0.253 / 60 * (time.time() - timeStart)
            velPub.publish(vel)
            rospy.sleep(0.1)

        rospy.sleep(2)
        velPub.publish(0)
        rospy.sleep(2)

        statePub.publish(2)  # 2 is for finished
        finishPub.publish(True)  # 1 is for finished
        rospy.sleep(10)


def main() -> None:
    """
    Static A, publish stuff
    """

    rospy.init_node("StaticA")
    staticA = StaticA()
    status = StatusPublisher("/status/staticA")
    status.starting()
    status.ready()
    drivingFlagTopic = rospy.get_param("/supervisor/driving_flag")
    rospy.Subscriber(drivingFlagTopic, Bool, staticA.drivingFlagCallback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status.running()
        if staticA.started:
            staticA.run()
            break

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
