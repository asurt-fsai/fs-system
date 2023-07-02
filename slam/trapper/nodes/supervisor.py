#!/usr/bin/env python
"""
indication for the interpreter that should be used to run the script.
"""
import rospy
from tf_helper.StatusPublisher import StatusPublisher
from trapper.supervisor_data_number_of_laps import RosTrapper


def main() -> None:
    """
    the main running program which includes the definition of
    all the publishers and subscribers

    Subscribers:
    the node subscribes to:
    1- odometry message from kiss_icp
    2- LandmarkArray from perceprtion node


    Publishers:
    the node publishes:
    1- Number of completed laps (roadstate message)
    2- clustered cones (landmarkArray)

    """
    rospy.init_node("slam_node")
    status = StatusPublisher("trapper")
    rosTrapper = RosTrapper()
    # ros_trapper.run()
    status.starting()
    status.ready()

    # Main loop
    mainRate = 10
    rateR = rospy.Rate(mainRate)
    while not rospy.is_shutdown():
        status.running()
        rateR.sleep()
        rosTrapper.publishSupervisordata()
        rosTrapper.runClustercones()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
