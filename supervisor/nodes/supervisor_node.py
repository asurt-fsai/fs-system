#!/usr/bin/python3
"""
.

"""
import rospy
from std_msgs.msg import Bool, Float32
from eufs_msgs.msg import CanState
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf_helper.StatusPublisher import StatusPublisher
from supervisor import Supervisor  # type: ignore


def main() -> None:
    """
    Main function.
    """

    rospy.init_node("supervisor")

    status = StatusPublisher("/status/supervisor")
    status.starting()

    rosCanCmdTopic = rospy.get_param("/ros_can/cmd")
    drivingFlagTopic = rospy.get_param("/supervisor/driving_flag")
    missionFLagTopic = rospy.get_param("/supervisor/mission_flag")

    isFinishedTopic = rospy.get_param("/finisher/is_finished")
    rosCanStateTopic = rospy.get_param("/ros_can/can_state")
    rosCanVelTopic = rospy.get_param("/ros_can/twist")
    controlVelTopic = rospy.get_param("/control/velocity")
    controlSteerTopic = rospy.get_param("/control/steering")

    markerTopic = rospy.get_param("/visualizer/markers")
    btnTopic = rospy.get_param("/visualizer/button")

    supervisor = Supervisor(
        rosCanCmdTopic, drivingFlagTopic, missionFLagTopic, markerTopic, btnTopic
    )

    rospy.Subscriber(rosCanStateTopic, CanState, supervisor.canStateCallback)
    rospy.Subscriber(isFinishedTopic, Bool, supervisor.isFinishedCallback)

    rospy.Subscriber(rosCanVelTopic, TwistWithCovarianceStamped, supervisor.currentVelCallback)
    rospy.Subscriber(controlVelTopic, Float32, supervisor.velCallback)
    rospy.Subscriber(controlSteerTopic, Float32, supervisor.steerCallback)

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        supervisor.run()
        rate.sleep()

        status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
