#!/usr/bin/env python3
"""
Simple Pure Pursuit Controller for lateral control of the vehicle
"""
import math

from typing import List, Tuple

import rospy
from nav_msgs.msg import Path
from tf_helper.TFHelper import TFHelper


BASELENGTH = rospy.get_param("/base_length", 2.5)  # [m] car length
LOOKAHEADCONSTANT = rospy.get_param("/look_ahead_constant", 2.5)  # look ahead constant


class WayPoints:
    """
    Class to store new waypoints to a list of waypoints and search for the suitable target point
    to follow with the pure pursuit algorithm
    """

    def __init__(self) -> None:
        """
        Parameters
        ----------
        xList : List[float]
            list of x coordinates of the waypoints

        yList : List[float]
            list of y coordinates of the waypoints

        oldNearestPointIndex : int
            index of the nearest point to the vehicle at the previous time step

        """
        self.waypoints = Path()
        self.points = self.waypoints.poses
        self.xList: List[float] = []
        self.yList: List[float] = []

    def add(self, waypointsMsg: Path) -> None:
        """
        Add each waypoint element to it's corrosponding list

        Points must be given in rear axle frame

        Parameters
        ----------
        waypoints : Path
            waypoint of the vehicle received from the path planner

        points : List[PoseStamped]
            list of waypoints of the vehicle received from the path planner
        """
        helper = TFHelper("control")

        self.waypoints = helper.transformMsg(waypointsMsg, "rear_axle")

        self.points = self.waypoints.poses

    def searchTargetIndex(self) -> Tuple[int, float]:
        """
        Search for the nearest point to the vehicle and calculate the distance between the vehicle


        Returns
        -------
        ind : int
            target point index choosen to follow from the waypoints list

        lookahead : float
            lookahead distance to the target point
        """

        lookAhead: float = LOOKAHEADCONSTANT

        self.xList = []
        self.yList = []
        ind = 0
        for index, _ in enumerate(self.waypoints.poses):

            self.xList.append(self.waypoints.poses[index].pose.position.x)
            self.yList.append(self.waypoints.poses[index].pose.position.y)

        while ind <= len(self.xList) - 1:

            distanceThisIndex = math.hypot(self.xList[ind], self.yList[ind])
            if distanceThisIndex >= lookAhead:
                break
            ind = ind + 1

        return ind, lookAhead

    def purepursuitSteercontrol(self, pind: int) -> Tuple[float, int]:
        """
        Calculate the steering angle to follow the target point

        Parameters
        ----------

        pind : int
            index of the nearest point to the vehicle at the previous time step


        Returns
        -------
        delta : float
            steering angle to follow the target point

        ind : int
            index of the target point
        """
        ind, lookAhead = self.searchTargetIndex()
        trajX: float = 0.0
        trajY: float = 0.0
        if pind >= ind:
            ind = pind
        if self.points != []:

            if ind < len(self.xList):
                trajX = self.xList[ind]
                trajY = self.yList[ind]

            else:  # toward goal
                trajX = self.points[-1].pose.position.x
                trajY = self.points[-1].pose.position.y
                ind = len(self.points) - 1

        alpha: float = math.atan2(trajY, trajX)

        delta: float = math.atan2(2.0 * BASELENGTH * math.sin(alpha) / lookAhead, 1.0)

        return delta, ind
