#!/usr/bin/env python3
import rospy
from planning_apf import PlanningRos


if __name__ == "__main__":
    rospy.init_node("planner_node")
    planner = PlanningRos("/carmaker/ObjectList", "/pathplanning/waypoints", is_ipg=True, plot=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        planner.run()
        rate.sleep()