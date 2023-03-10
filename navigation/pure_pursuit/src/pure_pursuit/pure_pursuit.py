#!/usr/bin/env python3
"""
Pure Pursuit Controller
"""
import math
from dataclasses import dataclass
from typing import List, Tuple

# import rospy
import numpy as np
from geometry_msgs.msg import Pose

# from ackermann_msgs.msg import AckermannDriveStamped
# import asurt_msgs.msg as asurt_msgs
# from std_msgs.msg import Path
from nav_msgs.msg import Path

DT = 0.1  # rospy.get_param("/time_step") # [s] time step
BASEWIDTH = 2.9  # rospy.get_param("/base_width") # [m] car length
GAINLOOKAHEAD = 0.5  # rospy.get_param("/gains/look_forward") # look forward gain
LOOKAHEADCONSTANT = 2.0  # rospy.get_param("/look_ahead_constant") # look ahead constant


@dataclass
class Position:
    """
    data class to store position of the vehicle
    """

    x: float
    y: float
    oldX: float
    oldY: float


class State:
    """
    doc string
    """

    def __init__(self, x: float, y: float, yaw: float, currentSpeed: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rearX = self.x - ((BASEWIDTH / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BASEWIDTH / 2) * math.sin(self.yaw))
        # self.oldX = x
        # self.oldY = y

    def update(self, currentState: Pose) -> None:
        """
        update the state of the vehicle
        """
        # position = Position(currentState.position.x, currentState.position.y, oldX, self.oldY)
        oldX = self.x
        oldY = self.y
        self.x = currentState.position.x
        self.y = currentState.position.y
        self.yaw = currentState.orientation.z
        self.currentSpeed = (
            math.sqrt((math.pow(self.x - oldX, 2) + math.pow(self.y - oldY, 2))) / DT
        )
        self.rearX = self.x - ((BASEWIDTH / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BASEWIDTH / 2) * math.sin(self.yaw))

    def calcDistance(self, pointX: float, pointY: float) -> float:
        """

        calculate the distance between the vehicle and the target point

        """
        distanceX = self.rearX - pointX
        distanceY = self.rearY - pointY
        return math.hypot(distanceX, distanceY)


class States:
    """
    State Class
    """

    def __init__(self) -> None:
        self.yList: List[float] = []
        self.xList: List[float] = []
        self.yaws: List[float] = []
        self.currentSpeeds: List[float] = []
        self.oldNearestPointIndex = None

    def add(self, state: State) -> None:
        """
        append state to the states list
        """
        self.xList.append(state.x)
        self.yList.append(state.y)
        self.yaws.append(state.yaw)
        self.currentSpeeds.append(state.currentSpeed)

    def statesCounter(self) -> int:
        """
        search state index from the states list
        """
        statesNumber: int = len(self.xList)
        return statesNumber


class WayPoints:
    """
    WayPoints Class
    """

    def __init__(self) -> None:
        self.waypoints = Path()
        self.xList: List[float] = []
        self.yList: List[float] = []
        self.oldNearestPointIndex: int = 0

    def add(self, waypoint: Pose) -> None:
        """
        add new waypoint to the waypoints list
        """
        self.waypoints = waypoint
        self.xList.append(self.waypoints.position.x)  # poses[-1].position.x)
        self.yList.append(self.waypoints.position.y)  # poses[-1].position.y)

    def searchTargetIndex(self, state: State) -> Tuple[int, float]:
        """
        search target point index from nearest point
        """
        # To speed up nearest point search, doing it at only first time
        if self.oldNearestPointIndex == 0:
            # search nearest point index
            distanceX = [state.rearX - icx for icx in self.xList]
            distanceY = [state.rearY - icy for icy in self.yList]
            distance = np.hypot(distanceX, distanceY)
            ind: int = int(np.argmin(distance))
            self.oldNearestPointIndex = ind
        else:
            ind = self.oldNearestPointIndex
            distanceThisIndex = state.calcDistance(self.xList[ind], self.yList[ind])
            while True:
                distanceNextIndex = state.calcDistance(self.xList[ind + 1], self.yList[ind + 1])
                if distanceThisIndex < distanceNextIndex:
                    break
                ind = ind + 1 if (ind + 1) < len(self.xList) else ind
                distanceThisIndex = distanceNextIndex
            self.oldNearestPointIndex = ind

        lookAhead: float = GAINLOOKAHEAD * state.currentSpeed + LOOKAHEADCONSTANT
        # update look ahead distance
        # search look ahead target point index
        # while lookAhead > state.calcDistance(self.xList[ind], self.yList[ind]):
        #     if (ind + 1) >= len(self.xList):
        #         break  # not exceed goal
        #     ind += 1

        return ind, lookAhead  # 3ayz a2asem de le 2 functions


def purepursuitSteercontrol(state: State, trajectory: WayPoints, pind: int) -> Tuple[float, int]:
    """
    pure pursuit steering control
    """
    ind, lookAhead = trajectory.searchTargetIndex(state)
    trajX: float = 0
    trajY: float = 0
    if pind >= ind:
        ind = pind

    if ind < len(trajectory.xList):
        trajX = trajectory.xList[ind]
        trajY = trajectory.yList[ind]
    else:  # toward goal
        trajX = trajectory.xList[-1]
        trajY = trajectory.yList[-1]
        ind = len(trajectory.xList) - 1

    alpha: float = math.atan2(trajY - state.rearY, trajX - state.rearX) - state.yaw

    delta: float = math.atan2(2.0 * BASEWIDTH * math.sin(alpha) / lookAhead, 1.0)

    return delta, ind
