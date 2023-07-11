#!/usr/bin/python3
"""
Static B
"""
import rospy
from std_msgs.msg import Float32, Bool, Int16
from tf_helper.StatusPublisher import StatusPublisher
import numpy as np
from std_srvs.srv import Trigger, TriggerRequest


class StaticB:
    """
    Static B
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

    @staticmethod
    def run() -> None:
        """
        Run Static B
        """
        rospy.loginfo("Starting Static B")
        velTopic = rospy.get_param("/control/velocity")
        isFinishedTopic = rospy.get_param("/finisher/is_finished")
        stateTopic = rospy.get_param("/state")
        ebsTopic = rospy.get_param("/ros_can/ebs")

        velPub = rospy.Publisher(velTopic, Float32, queue_size=10)
        statePub = rospy.Publisher(stateTopic, Int16, queue_size=10)
        finishPub = rospy.Publisher(isFinishedTopic, Bool, queue_size=10)

        rospy.sleep(2)
        vel = 2 * np.pi * 50 * 0.253 / 60  # 50 rpm
        velPub.publish(vel)
        rospy.sleep(10)

        ############################

        # CALLING EBS SERVICE HERE

        ############################
        rospy.wait_for_service(ebsTopic)
        # Create the connection to the service. Remember it's a Trigger service
        ebsService = rospy.ServiceProxy(ebsTopic, Trigger)
        # Create an object of the type TriggerRequest.
        # We nned a TriggerRequest for a Trigger service
        ebs = TriggerRequest()
        # Now send the request through the connection
        ebsService(ebs)
        ############################

        statePub.publish(2)  # 2 is for finished
        finishPub.publish(True)  # 1 is for finished


def main() -> None:
    """
    Static B, publish stuff
    """

    rospy.init_node("StaticB")
    staticB = StaticB()
    drivingFlagTopic = rospy.get_param("/supervisor/driving_flag")
    rospy.Subscriber(drivingFlagTopic, Bool, staticB.drivingFlagCallback)

    status = StatusPublisher("/status/staticB")
    status.starting()
    status.ready()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status.running()
        if staticB.started:
            staticB.run()
            break

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
