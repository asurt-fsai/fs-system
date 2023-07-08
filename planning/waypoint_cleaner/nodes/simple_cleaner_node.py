#!/usr/bin/env python3
"""_
Waypoints cleaner node to clean the waypoints from planning
"""
import rospy
from simple_waypoints_cleaner import SimpleWaypointsCleaner  # type: ignore[attr-defined]


def main() -> None:
    """
    Main function
    """
    rospy.init_node("simple_waypoints_cleaner_node")

    cleanWaypointsTopic = rospy.get_param("/planning_cleaner/clean_waypoints_topic")
    waypointsTopic = rospy.get_param("/planning_cleaner/waypoints_topic")
    frameId = rospy.get_param("/planning_cleaner/cleaner_frame_id")
    nPathsToKeep = rospy.get_param("/planning_cleaner/n_path_to_keep")
    radiusToMergeWaypoints = rospy.get_param("/planning_cleaner/radius_to_merge_path")
    minWaypointOccur = rospy.get_param("/planning_cleaner/min_waypoint_occur")

    cleaner = SimpleWaypointsCleaner(
        cleanWaypointsTopic,
        waypointsTopic,
        frameId,
        nPathsToKeep,
        radiusToMergeWaypoints,
        minWaypointOccur,
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        cleaner.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
