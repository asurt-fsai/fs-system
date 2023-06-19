#!/usr/bin/env python3
import rospy
from planning_apf import PlanningRos
from tf_helper.TFHelper import TFHelper

if __name__ == "__main__":
    rospy.init_node("planner_node")
    helper = TFHelper("planning")

    cones_topic = rospy.get_param("/planning/cones_topic")
    waypoints_topic = rospy.get_param("/planning/waypoints_topic")
    planning_frame_id = rospy.get_param("/planning/frame_id")

    planner = PlanningRos(
        cones_topic,
        waypoints_topic,
        helper,
        planning_frame_id,
        is_ipg=False,
        plot=True,
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        planner.run()
        rate.sleep()
