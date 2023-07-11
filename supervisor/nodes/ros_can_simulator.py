#!/usr/bin/python3
"""
.

"""
import rospy
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from supervisor.helpers.rosCanSimulator import RosCanSimulator


def main() -> None:
    """
    Main function.
    """

    rospy.init_node("ros_can_simulator")
    drivingFlagTopic = rospy.get_param("/supervisor/driving_flag")
    missionFLagTopic = rospy.get_param("/supervisor/mission_flag")
    rosCanCmdTopic = rospy.get_param("/ros_can/cmd")
    vcuCurrVelTopic = rospy.get_param("/vcu/curr_vel")
    vcuVelTopic = rospy.get_param("/vcu/control_vel")
    vcuSteerTopic = rospy.get_param("/vcu/control_steer")
    rosCanStateTopic = rospy.get_param("/ros_can/can_state")
    rosCanVelTopic = rospy.get_param("/ros_can/twist")

    rosCanSimulator = RosCanSimulator(
        15, vcuVelTopic, vcuSteerTopic, rosCanStateTopic, rosCanVelTopic
    )
    rospy.Subscriber(drivingFlagTopic, Bool, rosCanSimulator.drivingFlagCallback)
    rospy.Subscriber(missionFLagTopic, Bool, rosCanSimulator.missionFlagCallback)
    rospy.Subscriber(rosCanCmdTopic, AckermannDriveStamped, rosCanSimulator.cmdCallback)
    rospy.Subscriber(vcuCurrVelTopic, Float32, rosCanSimulator.currentVelCallback)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rosCanSimulator.run()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
