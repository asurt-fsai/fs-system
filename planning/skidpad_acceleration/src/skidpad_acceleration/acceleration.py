"""
This code generates a List of points representing waypoints along
a straight path, with x-coordinates set to 0 and y-coordinates
evenly spaced using linspace. It is useful for visualizing
or plotting trajectories and navigation systems.
"""
from typing import List
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped


class Acceleration:
    """
    This class generates a List of points representing waypoints along
    """

    def __init__(self, lenOfLine: float) -> None:
        """
        This method initializes the class with a length of the line
            Parameters:
            ----------
            lenOfLine: float
        """
        self.lenOfLine = lenOfLine

    def getWaypoints(self) -> List[PoseStamped]:
        """
        This method generates a List of points representing waypoints along
        Returns:
        ----------
        waypoint: list
        """
        waypoints = np.linspace(0, self.lenOfLine, int(self.lenOfLine / 0.1))
        waypoint = [(0, y) for y in waypoints]
        return waypoint

    def plotWaypoints(self, waypoints: List[PoseStamped]) -> None:
        """
        This method plots the waypoints
            Parameters:
            ----------
            waypoints: List
        """
        xValues = [point[0] for point in waypoints]
        yValues = [point[1] for point in waypoints]

        plt.plot(xValues, yValues, "ro-")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Waypoints")

        plt.xlim(min(xValues) - 1, max(xValues) + 1)
        plt.ylim(min(yValues) - 1, max(yValues) + 1)

        plt.show()
