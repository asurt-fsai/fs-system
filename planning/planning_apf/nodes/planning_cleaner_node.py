#!/usr/bin/env python3

"""_
initializes cleaner node
"""
from waypoints_cleaner import WaypointsCleaner
from planning_apf import PlanningRos
from tf_helper.TFHelper import TFHelper
from nav_msgs.msg import Path
import rospy


def planningCallback(msg):
    global helper, planningFrameId, waypoints
    to_rear = helper.transformMsg("rear_link", planningFrameId)
    cleaner = WaypointsCleaner(to_rear[0])
    pose = helper.transformMsg(planningFrameId, "map")
    cleaner.updatePosition(*pose)
    cleaner.addWaypoints(msg[1:])
    waypoints = cleaner.getWaypoints()


def main():
    global helper, planningFrameId, waypoints
    rospy.init_node("waypoints_cleaner_node")
    helper = TFHelper("planning")

    waypointsTopic = rospy.get_param("/planning/waypoints_topic")
    planningFrameId = rospy.get_param("/planning/frame_id")

    rospy.Subscriber(waypointsTopic, Path, planningCallback, queue_size=10)
    rospy.spin()


if __name__ == "__main__":
    main()
