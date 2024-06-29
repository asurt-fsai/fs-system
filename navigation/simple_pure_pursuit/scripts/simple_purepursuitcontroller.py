#!/usr/bin/env python3
"""
Simple Pure Pursuit Controller for lateral control of the vehicle
"""
import math

from typing import List, Tuple


from nav_msgs.msg import Path
from rclpy.node import Node

from tf_helper.TFHelper import TFHelper


class SimplePurePursuit:  # pylint: disable=too-many-instance-attributes
    """
    Class to run the simple pure pursuit controller
    """

    def __init__(self, node: Node) -> None:
        """
        Parameters
        ----------
        xList : List[float]
            list of x coordinates of the waypoints

        yList : List[float]
            list of y coordinates of the waypoints
        """
        self.node = node
        self.waypoints = Path()
        self.points = self.waypoints.poses
        self.xList: List[float] = []
        self.yList: List[float] = []

        self.helper = TFHelper("control")
        self.lookAhead: float = (
            self.node.get_parameter("control.look_ahead_constant")
            .get_parameter_value()
            .double_value
        )
        self.baseLength: float = (
            self.node.get_parameter("physical.car_base_length").get_parameter_value().double_value
        )
        self.frameId = (
            self.node.get_parameter("control.frame_id").get_parameter_value().string_value
        )
        # [m] car length

    def add(self, waypointsMsg: Path) -> None:
        """
        Get the waypoints from the waypoints node and transform them to the rear_link frame

        Parameters
        ----------
        waypointsMsg : Path
            list of waypoints to follow

        """

        self.waypoints = self.helper.transformMsg(waypointsMsg, "rear_link")

        self.points = waypointsMsg.poses

    def searchTargetindex(self) -> int:
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
            distanceThisindex = math.hypot(self.xList[ind], self.yList[ind])
            if distanceThisindex >= self.lookAhead:
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
        # print("Went Here to purepursuit")

        ind = self.searchTargetindex()
        traJx: float = 0.0
        traJy: float = 0.0
        if self.points != []:
            if ind < len(self.xList):
                traJx = self.xList[ind]
                traJy = self.yList[ind]

            else:  # toward goal
                traJx = self.points[-1].pose.position.x
                traJy = self.points[-1].pose.position.y
                ind = len(self.points) - 1

        alpha: float = math.atan2(traJy, traJx)

        delta: float = math.atan2(2.0 * self.baseLength * math.sin(alpha) / self.lookAhead, 1.0)

        return delta, ind
