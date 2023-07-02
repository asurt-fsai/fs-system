#!/usr/bin/env python3

"""_
initializes planning node
"""
from planning_apf import PlanningRos
from tf_helper.TFHelper import TFHelper
import rospy
import time


if __name__ == "__main__":
    rospy.init_node("planner_node")
    helper = TFHelper("planning")

    conesTopic = rospy.get_param("/planning/cones_topic")
    waypointsTopic = rospy.get_param("/planning/waypoints_topic")
    planningFrameId = rospy.get_param("/planning/frame_id")
    kAttractive = rospy.get_param("/planning/kAttractive")
    kRepulsive = rospy.get_param("/planning/kRepulsive")
    repulsiveRadius = rospy.get_param("/planning/repulsiveRadius")
    stepSize = rospy.get_param("/planning/stepSize")
    maxIterations = rospy.get_param("/planning/maxIterations")
    goalThreshold = rospy.get_param("/planning/goalThreshold")

    planner = PlanningRos(
        conesTopic,
        waypointsTopic,
        helper,
        planningFrameId,
        kAttractive,
        kRepulsive,
        repulsiveRadius,
        stepSize,
        maxIterations,
        goalThreshold,
        isIpg=False,
        plot=True,
    )
    # conesLength = planner.getConesLength()
    # print("conesLength: ", conesLength)
    rate = rospy.Rate(10)
    counter = 0
    while not rospy.is_shutdown():
        if len(planner.allCones) > 0:
            counter += 1
            planner.run(counter)

        rate.sleep()
