#!/usr/bin/python3

"""
Node to run the accelration planning algorithm

"""
import rospy
from tf_helper.StatusPublisher import StatusPublisher
from acceleration.acc_planning import AccPlanner


def main() -> None:
    """
    Main Loop
    """
    # Initialize ROS node
    rospy.init_node("acceleration_planning")
    status = StatusPublisher("/status/acceleration_planning")

    status.starting()
    trackWidth = rospy.get_param("/physical/track_width")
    lengthOfPath = rospy.get_param("/acceleration_planning/length_of_path")
    pathResolution = rospy.get_param("/acceleration_planning/path_resolution")
    filterRange = rospy.get_param("/acceleration_planning/filter_range")
    topics = {
        "path": rospy.get_param("/acceleration_planning/path_topic"),
        "cones": rospy.get_param("/acceleration_planning/cones_topic"),
        "map": rospy.get_param("/acceleration_planning/map_topic"),
        "odom": rospy.get_param("/acceleration_planning/odom_topic"),
    }

    accPlanner = AccPlanner(
        trackWidth=trackWidth,
        lengthOfPath=lengthOfPath,
        pathResolution=pathResolution,
        filterRange=filterRange,
        topics=topics,
    )
    accPlanner.start()
    status.ready()

    # Main loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        out = accPlanner.plan()
        if out is None:
            continue
        # Publish heartbeat to show the module is running
        status.running()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
