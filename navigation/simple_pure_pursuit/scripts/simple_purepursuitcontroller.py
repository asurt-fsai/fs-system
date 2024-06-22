#!/usr/bin/env python3
"""
Simple Pure Pursuit Controller for lateral control of the vehicle
"""
import math

from typing import List, Tuple


from nav_msgs.msg import Path
from rclpy.node import Node

from tf_helper.TFHelper import TFHelper


class SimplePurePursuit:
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
        self.x_list: List[float] = []
        self.y_list: List[float] = []

        self.helper = TFHelper("control")
        self.look_ahead:float = self.node.get_parameter("/control/look_ahead_constant")\
                                                              .get_parameter_value().double_value
        self.base_length:float = self.node.get_parameter("/physical/car_base_length")\
                                                              .get_parameter_value().double_value
        # [m] car length

    def add(self, waypoints_msg: Path) -> None:
        """
        Get the waypoints from the waypoints node and transform them to the rear_link frame

        Parameters
        ----------
        waypointsMsg : Path
            list of waypoints to follow

        """

        self.waypoints = self.helper.transformMsg(waypoints_msg, "rear_link")

        self.points = waypoints_msg.poses

    def search_targetindex(self) -> int:
        """
        Search for the nearest point to the vehicle and calculate the distance between the vehicle

        Returns
        -------
        ind : int
            target point index choosen to follow from the waypoints list
        """

        self.x_list = []
        self.y_list = []
        ind = 0
        for index, _ in enumerate(self.waypoints.poses):
            self.x_list.append(self.waypoints.poses[index].pose.position.x)
            self.y_list.append(self.waypoints.poses[index].pose.position.y)

        while ind <= len(self.x_list) - 1:
            distance_thisindex = math.hypot(self.x_list[ind], self.y_list[ind])
            if distance_thisindex >= self.look_ahead:
                break
            ind = ind + 1

        return ind

    def purepursuit_teercontrol(self) -> Tuple[float, int]:
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

        ind = self.search_targetindex()
        traj_x: float = 0.0
        traj_y: float = 0.0
        if self.points != []:
            if ind < len(self.x_list):
                traj_x = self.x_list[ind]
                traj_y = self.y_list[ind]

            else:  # toward goal
                traj_x = self.points[-1].pose.position.x
                traj_y = self.points[-1].pose.position.y
                ind = len(self.points) - 1

        alpha: float = math.atan2(traj_y, traj_x)

        delta: float = math.atan2(2.0 * self.base_length * math.sin(alpha) / self.look_ahead, 1.0)

        return delta, ind
    