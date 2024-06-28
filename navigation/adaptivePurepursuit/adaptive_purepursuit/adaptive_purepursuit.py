"""
This module contains the implementation of the AdaptivePurePursuit class
for the kinematic bicycle model.

The AdaptivePurePursuit class provides methods for calculating the steering angle
and speed control for the kinematic bicycle model using the adaptive pure pursuit algorithm.

Classes:
- AdaptivePurePursuit: Adaptive Pure Pursuit class for the kinematic bicycle model.

Dataclasses:
- GainParams: Dataclass for PID controller gains.
- SpeedLimits: Dataclass for speed limits.
- Constants: Constants for the adaptive pure pursuit algorithm.
"""

from dataclasses import dataclass
import math
from typing import List, Tuple
from rclpy import Node


class AdaptivePurePursuit:  # pylint: disable=too-many-instance-attributes
    """
    Adaptive Pure Pursuit class for the kinematic bicycle model.

    Args:
        node: rclpy node object

    Returns:
        None
    """

    def __init__(self, node: Node) -> None:
        @dataclass
        class GainParams:
            """Dataclass for PID controller gains."""

            proportional: float
            integral: float
            differential: float
            lookahead: float
            prevError: float = 0.0
            errorSum: float = 0.0

        @dataclass
        class SpeedLimits:
            """Dataclass for speed limits."""

            minimum: float
            maximum: float

        @dataclass
        class Constants:
            """Constants for the adaptive pure pursuit algorithm."""

            speed: float
            lookahead: float

        self.targetSpeed = 0.0
        self.waypoints: List[Tuple[float, float]] = []
        self.firstFlag = True
        self.targetIndex = 0
        self.steeringAngle = 0.0
        self.lookaheadDistance = 0.0
        self.state = [0.0, 0.0, 0.0, 0.0]  # 0:x , 1:y , 2:yaw , 3:velocity
        self.node = node
        self.gains = self.node.get_parameter_or("/gains", GainParams(0.0, 0.0, 0.0, 0.0))
        self.speedLimits = self.node.get_parameter_or("/speed", SpeedLimits(0.0, 0.0))
        self.constants = self.node.get_parameter_or("/constants", Constants(0.0, 0.0))
        self.deltaT = self.node.get_parameter_or("/time_step", 0.0)
        self.gains.prevError = 0.0
        self.gains.errorSum = 0.0

    @staticmethod
    def calculateDistance(point1: Tuple[float, float], point2: Tuple[float, float]) -> float:
        """
        Calculate the distance between two points in 2D space.

        args:
            point1: list
            point2: list

        returns:
            distance: float
        """
        deltaX = point2[0] - point1[0]
        deltaY = point2[1] - point1[1]
        return math.sqrt(deltaX**2 + deltaY**2)

    def searchTargetpoint(self) -> int:
        """
        Search for the target point in the waypoints list.

        args:
            None

        returns:
            targetIndex: int
        """
        minDistance = float("inf")
        statePoint = (self.state[0], self.state[1])
        if self.firstFlag:
            for i, waypoint in enumerate(self.waypoints):
                distance = self.calculateDistance(statePoint, waypoint)
                if distance < minDistance:
                    minDistance = distance
                    self.targetIndex = i
                    self.firstFlag = False

        for i in range(self.targetIndex, len(self.waypoints) - 1):
            distance = self.calculateDistance(statePoint, self.waypoints[i])
            if distance > self.lookaheadDistance:
                self.targetIndex = i
                break
        return self.targetIndex

    def adaptivePurepursuit(self) -> float:
        """
        Adaptive lookahead distance calculation for the pure pursuit algorithm.

        args:
            None

        returns:
            steeringAngle: float
        """
        self.lookaheadDistance = self.state[3] * self.gains.lookahead + self.constants.lookahead
        self.targetIndex = self.searchTargetpoint()
        targetWaypoint = self.waypoints[self.targetIndex]
        targetX, targetY = targetWaypoint
        deltaX = targetX - self.state[0]
        deltaY = targetY - self.state[1]
        alpha = math.atan2(deltaY, deltaX) - self.state[2]
        lookaheadAngle = math.atan2(2 * 0.5 * math.sin(alpha) / self.lookaheadDistance, 1)
        self.steeringAngle = math.degrees(lookaheadAngle)
        self.steeringAngle = max(-0.5, min(0.5, lookaheadAngle))
        return self.steeringAngle

    def speedControl(self, steeringAngle: float) -> float:
        """
        Speed control function for the kinematic bicycle model.

        args:
            steeringAngle: float

        returns:
            targetSpeed: float
        """
        self.targetSpeed = self.constants.speed / (abs(steeringAngle) + 0.001)
        self.targetSpeed = min(self.targetSpeed, self.speedLimits.maximum)
        self.targetSpeed = max(self.targetSpeed, self.speedLimits.minimum)
        return self.targetSpeed

    def pidController(self, steering: float) -> float:
        """
        PID controller for the kinematic bicycle model.

        args:
            steering: float

        returns:
            controlSignal: float
        """
        self.targetSpeed = self.speedControl(steering)
        error = self.targetSpeed - self.state[3]
        pTerm = self.gains.proportional * error
        self.gains.errorSum += error
        iTerm = self.gains.integral * self.gains.errorSum
        dTerm = self.gains.differential * (error - self.gains.prevError) / self.deltaT
        controlSignal: float = pTerm + iTerm + dTerm
        self.gains.prevError = error
        controlSignal = max(-1.0, min(1.0, controlSignal))
        return controlSignal
