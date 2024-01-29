#!/usr/bin/env python3
"""
Simple Pure Pursuit Controller for lateral control of the vehicle
"""
import math

from typing import List, Tuple

import rclpy
from nav_msgs.msg import Path
#from tf_helper.TFHelper import TFHelper




class SimplePurePursuit:
    """
    Class to run the simple pure pursuit controller
    """

    def __init__(self) -> None:
        """
        Parameters
        ----------
        xList : List[float]
            list of x coordinates of the waypoints

        yList : List[float]
            list of y coordinates of the waypoints
        """
        self.waypoints = Path()
        self.points = self.waypoints.poses
        self.xList: List[float] = []
        self.yList: List[float] = []
     
        # self.helper = TFHelper("control")
        self.lookAhead = 6.1#rclpy.Parameter.get_parameter_value('/control/look_ahead_constant')
        self.baseLength = 2.5#rclpy.Parameter.get_parameter_value('/physical/car_base_length')  # [m] car length
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

        #self.waypoints = self.helper.transformMsg(waypointsMsg, "rear_link")
        print("Went Here")
        self.waypoints = waypointsMsg
        self.points = waypointsMsg.poses

    def searchTargetIndex(self) -> int:
        """
        Search for the nearest point to the vehicle and calculate the distance between the vehicle

        Returns
        -------
        ind : int
            target point index choosen to follow from the waypoints list
        """

        self.xList = []
        self.yList = []
        ind = 0
        for index, _ in enumerate(self.waypoints.poses):
            self.xList.append(self.waypoints.poses[index].pose.position.x)
            self.yList.append(self.waypoints.poses[index].pose.position.y)

        while ind <= len(self.xList) - 1:
            distanceThisIndex = math.hypot(self.xList[ind], self.yList[ind])
            if distanceThisIndex >= self.lookAhead:
                break
            ind = ind + 1

        return ind

    def purepursuitSteercontrol(self) -> Tuple[float, int]:
        """
        Calculate the steering angle to follow the target point

        Returns
        -------
        delta : float
            steering angle to follow the target point
        ind : int
            target point index choosen to follow from the waypoints list
        """
        #print("Went Here to purepursuit")
        
        ind = self.searchTargetIndex()
        trajX: float = 0.0
        trajY: float = 0.0
        if self.points != []:
            if ind < len(self.xList):
                trajX = self.xList[ind]
                trajY = self.yList[ind]

            else:  # toward goal
                trajX = self.points[-1].pose.position.x
                trajY = self.points[-1].pose.position.y
                ind = len(self.points) - 1

        alpha: float = math.atan2(trajY, trajX)

        delta: float = math.atan2(2.0 * self.baseLength * math.sin(alpha) / self.lookAhead, 1.0)

        return delta, ind
