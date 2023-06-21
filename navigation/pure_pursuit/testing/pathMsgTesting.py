# pylint: disable=all
# mypy: ignore-errors
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

# from pure_pursuit import WayPoints, purepursuitSteercontrol,State
BASEWIDTH = 2.9
from typing import List, Tuple
from dataclasses import dataclass
from geometry_msgs.msg import Pose
import numpy as np


@dataclass
class Position:
    """
    data class to store position of the vehicle
    """

    x: float
    y: float


class State:
    """
    state of the vehicle received from SLAM
    """

    def __init__(self, position: Position, yaw: float, currentSpeed: float = 0.0) -> None:
        """
        parameters
        ----------
        x : float
            x coordinate of the vehicle

        y : float
            y coordinate of the vehicle

        yaw : float
            yaw of the vehicle

        currentSpeed : float
            current speed of the vehicle

        rearX : float
            x coordinate of the rear of the vehicle

        rearY : float
            y coordinate of the rear of the vehicle
        """
        self.position: Position = Position(position.x, position.y)
        self.yaw: float = yaw
        self.currentSpeed: float = currentSpeed
        self.rearX: float = self.position.x - ((BASEWIDTH / 2) * math.cos(self.yaw))
        self.rearY: float = self.position.y - ((BASEWIDTH / 2) * math.sin(self.yaw))
        self.poseList: List[Tuple[float, float, float]] = []

    def update(self, currentState: Pose) -> None:
        """
        update the state of the vehicle to the new state

        Parameters
        ----------
        currentState : Pose
            new state of the vehicle received from SLAM
        """

        self.position.x = currentState.position.x
        self.position.y = currentState.position.y
        self.yaw = currentState.orientation.z
        self.currentSpeed = currentState.orientation.x
        self.rearX = self.position.x - ((BASEWIDTH / 2) * math.cos(self.yaw))
        self.rearY = self.position.y - ((BASEWIDTH / 2) * math.sin(self.yaw))
        self.poseList.append((self.position.x, self.position.y, self.yaw))

    def calcDistance(self, pointX: float, pointY: float) -> float:
        """
        calculate the distance between the rear of the vehicle and a point

        Parameters
        ----------
        pointX : float
            x coordinate of the point

        pointY : float
            y coordinate of the point

        Returns
        -------
        distance : float
            distance between the rear of the vehicle and the point

        """
        distanceX: float = self.rearX - pointX
        distanceY: float = self.rearY - pointY
        distance: float = math.hypot(distanceX, distanceY)

        return distance


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
        self.oldNearestPointIndex: int = 0
        self.firstLoop: bool = False

    def add(self, waypointsMsg: Path) -> None:
        """
        Add each waypoint element to it's corrosponding list

        Parameters
        ----------
        waypoints : Pose
            waypoint of the vehicle received from the path planner
        """
        self.waypoints = waypointsMsg
        self.firstLoop = False
        # self.xlist = waypoints.poses[0].pose.position.x
        # self.xList.append(waypoints.position.x)
        # self.yList.append(waypoints.position.y)

    def searchTargetIndex(self, state: State) -> Tuple[int, float]:
        """
        Search for the nearest point to the vehicle and calculate the distance between the vehicle

        Parameters
        ----------
        state : State
            current state of the vehicle

        Returns
        -------
        ind : int
            target point index choosen to follow from the waypoints list

        lookahead : float
            lookahead distance to the target point
        """

        lookAhead: float = 2.0  # LOOKAHEADCONSTANT

        if self.firstLoop is False:
            # search nearest point index
            for index, _ in enumerate(self.waypoints.poses):
                # Extracting and storing X and Y coordinates seperately in a list
                # to get minimum distance in first loop only

                self.xList.append(self.waypoints.poses[index].pose.position.x)
                self.yList.append(self.waypoints.poses[index].pose.position.y)
            distanceX = [state.rearX - icx for icx in self.xList]
            distanceY = [state.rearY - icy for icy in self.yList]
            distance = np.hypot(distanceX, distanceY)
            if len(distance) != 0:
                ind: int = int(np.argmin(distance))
                self.oldNearestPointIndex = ind
                self.firstLoop = True

        else:
            ind = self.oldNearestPointIndex
            distanceThisIndex = state.calcDistance(self.xList[ind], self.yList[ind])
            # distanceThisIndex = state.calcDistance(
            #     self.points[ind].pose.position.x, self.points[ind].pose.position.y
            # )
            while ind < len(self.xList) - 1:
                distanceNextIndex = state.calcDistance(self.xList[ind + 1], self.yList[ind + 1])
                # distanceNextIndex = state.calcDistance(
                #     self.points[ind + 1].pose.position.x, self.points[ind + 1].pose.position.y
                # )
                if distanceThisIndex < lookAhead:
                    ind = ind + 1

                else:

                    break

                distanceThisIndex = distanceNextIndex
            self.oldNearestPointIndex = ind

        return ind, lookAhead


rospy.init_node("pathMsgTesting", anonymous=True)
pathPub = rospy.Publisher("/path", Path, queue_size=10)
path = Path()
path.header.frame_id = "map"
path.header.stamp = rospy.Time.now()
point = path.poses
pose = PoseStamped()
pose.pose.position.x = 1.0
pose.pose.position.y = 2.0
pose.pose.orientation.z = 3.0
pose2 = PoseStamped()
pose2.pose.position.x = 4.0
pose2.pose.position.y = 5.0
pose2.pose.orientation.z = 6.0
path.poses.append(pose)
path.poses.append(pose2)
traj = WayPoints()
traj.waypoints = path
traj.searchTargetIndex(state=State(position=Position(0, 0), yaw=0))
if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            pathPub.publish(path)
            rospy.sleep(1)
            print((traj.searchTargetIndex(state=State(position=Position(0, 0), yaw=0))))
            print(traj.xList, traj.yList)
    except rospy.ROSInterruptException:
        pass
