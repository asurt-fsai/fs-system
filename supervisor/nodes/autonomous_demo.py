#!/usr/bin/python3
"""
Autonomous Demo
"""
import time
import rospy
from std_msgs.msg import Float32, Bool, Int16
from tf_helper.StatusPublisher import StatusPublisher
from std_srvs.srv import Trigger, TriggerRequest


class AutonomousDemo:
    """
    Autonomous Demo
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
        self.currentVel = 0.0
        self.timePrevStep = time.time()
        self.dist = 0.0
        self.drevT = 0.0

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
        Run  Autonomous Demo
        """
        rospy.loginfo("Starting Autonomous Demo")
        velTopic = rospy.get_param("/control/velocity")
        isFinishedTopic = rospy.get_param("/finisher/is_finished")
        stateTopic = rospy.get_param("/state")
        steerTopic = rospy.get_param("/control/steering")
        vcuCurrVelTopic = rospy.get_param("/vcu/curr_vel")

        rospy.Subscriber(vcuCurrVelTopic, Float32, self.currentVelCallback)
        velPub = rospy.Publisher(velTopic, Float32, queue_size=10)
        steerPub = rospy.Publisher(steerTopic, Float32, queue_size=10)
        statePub = rospy.Publisher(stateTopic, Int16, queue_size=10)
        finishPub = rospy.Publisher(isFinishedTopic, Int16, queue_size=10)
        maxSteer = float(rospy.get_param("/maxSteer"))

        rospy.sleep(2)
        statePub.publish(1)  # 1 is for driving
        steerPub.publish(-maxSteer)
        rospy.sleep(10)
        steerPub.publish(maxSteer)
        rospy.sleep(10)
        steerPub.publish(0)
        rospy.sleep(10)

        while self.dist < 10:
            self.dist += self.currentVel * self.drevT
            velPub.publish(1.0)

        rospy.sleep(5)
        velPub.publish(0.0)
        rospy.sleep(5)

        self.dist = 0.0
        while self.dist < 10:
            self.dist += self.currentVel * self.drevT
            velPub.publish(1.0)

        ############################
        # CALLING EBS SERVICE HERE
        ############################
        rospy.wait_for_service("/vcu_EBS")
        # Create the connection to the service. Remember it's a Trigger service
        ebsService = rospy.ServiceProxy("/vcu_EBS", Trigger)
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
    Autonomous Demo, publish stuff
    """

    rospy.init_node("autonomous_demo")
    status = StatusPublisher("/status/autonomous_demo")
    autonomousDemo = AutonomousDemo()
    drivingFlagTopic = rospy.get_param("/supervisor/driving_flag")
    rospy.Subscriber(drivingFlagTopic, Bool, autonomousDemo.drivingFlagCallback)

    status.starting()
    status.ready()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        status.running()
        if autonomousDemo.started:
            autonomousDemo.run()
            break

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
