#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Path calculation class.

Description: Last step in Pathing pipeline
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, cast

import numpy as np
from icecream import ic  # pylint: disable=unused-import

from src.calculate_path.path_calculator_helpers import (
    PathCalculatorHelpers,
)

# from planning_centerline_calculation.calculate_path.path_parameterization import PathParameterizer
from src.types_file.types import BoolArray, FloatArray, IntArray
from src.utils.cone_types import ConeTypes
from src.utils.math_utils import (
    angleFrom2dVector,
    circleFit,
    normalizeLastAxis,
    rotate,
    traceDistanceToNext,
    unit2dVectorFromAngle,
    vecAngleBetween,
    angleToVector,
)

@dataclass
class PathCalculationInput:
    """Dataclass holding calculation variables."""

    # pylint: disable=too-many-instance-attributes
    leftCones: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    rightCones: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    leftToRightMatches: IntArray = field(default_factory=lambda: np.zeros(0, dtype=int))
    rightToLeftMatches: IntArray = field(default_factory=lambda: np.zeros(0, dtype=int))
    positionGlobal: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    # directionGlobal: FloatArray = field(default_factory=lambda: np.array([1, 0]))
    directionGlobal: np.float_ = np.float_(0)
    globalPath: Optional[FloatArray] = field(default=None)


@dataclass
class PathCalculationScalarValues:
    """Class holding scalar values of a path calculator."""

    maximalDistanceForValidPath: float
    mpcPathLength: float = 30
    mpcPredictionHorizon: int = 40


class CalculatePath:
    """
    Class that takes all path calculation responsibilities after the cones have been
    matched.
    """

    def __init__(
        self,
        maximalDistanceForValidPath: float,
        mpcPathLength: float,
        mpcPredictionHorizon: int,
    ):
        """
        Init method.

        Args:
            smoothing, predict_every, max_deg: Arguments for cone fitting.
            maximal_distance_for_valid_path: Maximum distance for a valid path. If the
                calculated path has a minimum distance from the car that is larger than
                this value, the path is not valid, and the previously calculated path is
                used.
        """
        self.input = PathCalculationInput()
        self.scalars = PathCalculationScalarValues(
            maximalDistanceForValidPath=maximalDistanceForValidPath,
            mpcPathLength=mpcPathLength,
            mpcPredictionHorizon=mpcPredictionHorizon,
        )
        self.pathCalculatorHelpers = PathCalculatorHelpers()

        self.previousPaths = self.calculateInitialPath()
        # self.mpcPaths = []
        self.pathIsTrivialList: List[bool] = []
        self.pathUpdates: List[FloatArray] = []

    def calculateInitialPath(self) -> FloatArray:
        """
        Calculate the initial path.
        """

        return self.pathCalculatorHelpers.calculateAlmostStraightPath()

    def setNewInput(self, newInput: PathCalculationInput) -> None:
        """Update the state of the calculation."""
        self.input = newInput

    def calculateTrivialPath(self) -> FloatArray:
        "Calculate a path that points straight from the car position and direction"
        originPath = self.pathCalculatorHelpers.calculateAlmostStraightPath()[1:]
        yaw = self.input.directionGlobal
        pathRotated: FloatArray = rotate(originPath, yaw)  # type: ignore
        finalTrivialPath: FloatArray = pathRotated + self.input.positionGlobal
        return finalTrivialPath

    def numberOfMatchesOnOneSide(self, side: ConeTypes) -> int:
        """
        The matches array contains the index of the matched cone of the other side.
        If a cone does not have a match the index is set -1. This method finds how
        many cones actually have a match (the index of the match is not -1)
        """
        assert side in (ConeTypes.left, ConeTypes.right)
        matchesOfSide = (
            self.input.leftToRightMatches
            if side == ConeTypes.left
            else self.input.rightToLeftMatches
        )
        returnValue: int = np.sum(matchesOfSide != -1)
        return returnValue

    def sideScore(self, side: ConeTypes) -> tuple[int, int]:
        """
        Pick side with most matches, if both same number of matches, pick side
        where the indices increase the most.
        """
        matchesOfSide = (
            self.input.leftToRightMatches
            if side == ConeTypes.left
            else self.input.rightToLeftMatches
        )
        matchesOfSideFiltered = matchesOfSide[matchesOfSide != -1]
        nMatches = len(matchesOfSideFiltered)
        nIndicesSum = matchesOfSideFiltered.sum()

        # first pick side with most matches, if both same number of matches, pick side
        # where the indices increase the most
        return nMatches, nIndicesSum

    def selectSideToUse(self) -> Tuple[FloatArray, IntArray, FloatArray]:
        "Select the main side to use for path calculation"

        sideToPick = max([ConeTypes.left, ConeTypes.right], key=self.sideScore)

        sideToUse, matchesToOtherSide, otherSideCones = (
            (
                self.input.leftCones,
                self.input.leftToRightMatches,
                self.input.rightCones,
            )
            if sideToPick == ConeTypes.left
            else (
                self.input.rightCones,
                self.input.rightToLeftMatches,
                self.input.leftCones,
            )
        )
        return sideToUse, matchesToOtherSide, otherSideCones

    def calculateCenterlinePointsOfMatches(
        self,
        sideToUse: FloatArray,
        matchesToOtherSide: IntArray,
        matchOnOtherSide: FloatArray,
    ) -> FloatArray:
        """
        Calculate the basis of the new path by computing the middle between one side of
        the track and its corresponding match. If there are not enough cones with
        matches, the path from the previous calculation is used.
        """
        centerAlongMatchConnection = (sideToUse + matchOnOtherSide) / 2
        centerAlongMatchConnection = centerAlongMatchConnection[matchesToOtherSide != -1]

        # need at least 2 points for path calculation
        if len(centerAlongMatchConnection) < 2:
            centerAlongMatchConnection = self.previousPaths[-15:]

        return centerAlongMatchConnection

    def overwritePathIfItIsTooFarAway(self, pathUpdate: FloatArray) -> FloatArray:
        """
        If for some reason the calculated path is too far away from the position of the
        car (e.g. because of a bad sorting), the previously calculated path is used
        """
        minDistanceToPath = np.linalg.norm(self.input.positionGlobal - pathUpdate, axis=-1).min()
        if minDistanceToPath > self.scalars.maximalDistanceForValidPath:
            pathUpdate = self.previousPaths[-15:]
        return pathUpdate

    def extendPath(self, pathUpdate: FloatArray) -> FloatArray:
        """
        If the path is not long enough, extend it with the path with a circular arc
        """

        ## find the length of the path in front of the car

        # find angle to each point in the path
        carToPath = pathUpdate - self.input.positionGlobal
        maskPathIsInFrontOfCar = np.dot(carToPath, angleToVector(self.input.directionGlobal)) > 0
        # as soon as we find a point that is in front of the car, we can mark all the
        # points after it as being in front of the car
        for i, value in enumerate(maskPathIsInFrontOfCar.copy()):
            if value:
                maskPathIsInFrontOfCar[i:] = True
                break

        maskPathIsInFrontOfCar[-10:] = True

        if not maskPathIsInFrontOfCar.any():
            return pathUpdate

        pathInfrontOfCar = pathUpdate[maskPathIsInFrontOfCar]

        cumPathLength = traceDistanceToNext(pathInfrontOfCar).cumsum()
        # finally we get the length of the path in front of the car
        pathLength = cumPathLength[-1]

        if pathLength > self.scalars.mpcPathLength:
            return pathUpdate

        # select n last points of the path and estimate the circle they form
        relevantPath = pathInfrontOfCar[-10:]
        try:
            centerX, centerY, radius = circleFit(relevantPath)
            center = np.array([centerX, centerY])

            radiusToUse = min(max(radius, 10), 100)
        except ZeroDivisionError:
            radiusToUse = 100

        if radiusToUse < 80:
            # ic(center_x, center_y, radius_to_use)
            relevantPathCentered = relevantPath - center
            # find the orientation of the path part, to know if the circular arc should be
            # clockwise or counterclockwise
            threePoints = relevantPathCentered[[0, int(len(relevantPathCentered) / 2), -1]]

            # https://en.wikipedia.org/wiki/Curve_orientation#Orientation_of_a_simple_polygon
            homogeneousPoints = np.column_stack((np.ones(3), threePoints))
            orientation = np.linalg.det(homogeneousPoints)
            orientationSign = np.sign(orientation)

            # create the circular arc
            startAngle = float(angleFrom2dVector(threePoints[2]))  # + orientationSign * np.pi/4
            endAngle = startAngle + orientationSign * np.pi
            newPointsAngles = np.linspace(startAngle, endAngle)
            newPointsRaw = unit2dVectorFromAngle(newPointsAngles) * radiusToUse

            newPoints = newPointsRaw - newPointsRaw[0] + pathUpdate[-1]
            # ic(new_points)
            # to avoid overlapping when spline fitting, we need to first n points
        else:
            secondLastPoint = pathUpdate[-2]
            lastPoint = pathUpdate[-1]
            direction = lastPoint - secondLastPoint
            direction = direction / np.linalg.norm(direction)
            newPoints = lastPoint + direction * np.arange(30)[:, None]

        newPoints = newPoints[1:]
        return np.row_stack((pathUpdate, newPoints))

    def createPathForMpcFromPathUpdate(self, pathUpdate: FloatArray) -> FloatArray:
        """
        Calculate the path for MPC from the path update. The path update is the basis of
        the new path.

        First a linear path is added at the end of the path update. This ensures that
        the path is long enough for MPC. Otherwise we would have to use spline extrapolation
        to get a path that is long enough, however polynomial extrapolation is not stable
        enough for our purposes.

        Then the path is fitted again as a spline. Because we have now added the linear
        part we can be sure that no polynomial extrapolation will be used.

        Then any path behind the car is removed.

        Finally the path is trimmed to the correct length, as desired from MPC.

        Args:
            path_update: The basis of the new path

        Returns:
            The path for MPC
        """
        pathConnectedToCar = self.connectPathToCar(pathUpdate)
        pathAfterIncreasingPoints = self.increasePoints(pathConnectedToCar)
        pathWithEnoughLength = self.extendPath(pathAfterIncreasingPoints)
        pathWithNoPathBehindCar = self.removePathBehindCar(pathWithEnoughLength)

        pathWithLengthForMpc = self.removePathNotInPredictionHorizon(pathWithNoPathBehindCar)

        return pathWithLengthForMpc

    def costMpcPathStart(self, pathLengthFixed: FloatArray) -> FloatArray:
        """
        Cost function for start of MPC path. The cost is based on the distance from the
        car to the calculated path. Mission specific cost functions can be added here.
        """

        distanceCost: FloatArray = np.linalg.norm(
            self.input.positionGlobal - pathLengthFixed, axis=1
        )
        return distanceCost

    def increasePoints(self, path: FloatArray) -> FloatArray:
        """
        Inserts new points in a path array at locations exceeding a distance threshold.

        Args:
            path: A NumPy array representing the path coordinates (x, y).

        Returns:
            A new NumPy array with interpolated points added.
        """

        maxDistance = 1  # Distance threshold for inserting points
        flagToRepeat = 0
        newPath = list(path[:1])  # Start with the first point in the new path
        for i in range(len(path) - 1):  # Iterate over pairs of points (avoiding first and last)
            point1 = path[i]
            point2 = path[i + 1]
            distance = np.linalg.norm(point1 - point2)  # Efficient distance calculation

            if distance > maxDistance * 2:
                flagToRepeat = 1

            if distance > maxDistance:
                newPoint = (point1 + point2) / 2  # Midpoint between points
                newPath.append(newPoint)

            newPath.append(point2)  # Append the current point

        if flagToRepeat:
            return self.increasePoints(np.array(newPath))

        return np.array(newPath)

    def connectPathToCar(self, pathUpdate: FloatArray) -> FloatArray:
        """
        Connect the path update to the current path of the car. This is done by
        calculating the distance between the last point of the path update and the
        current position of the car. The path update is then shifted by this distance.
        """
        distanceToFirstPoint = np.linalg.norm(self.input.positionGlobal - pathUpdate[0])

        carToFirstPoint = pathUpdate[0] - self.input.positionGlobal

        vectorVehicleDirection = angleToVector(self.input.directionGlobal)

        angleToFirstPoint = vecAngleBetween(carToFirstPoint+0.00001, vectorVehicleDirection+0.00001)

        # there is path behind car or start is close enough
        if distanceToFirstPoint < 0.5 or angleToFirstPoint > np.pi / 2:
            return pathUpdate

        newPoint = self.input.positionGlobal + normalizeLastAxis(carToFirstPoint[None])[0] * 0.2

        pathUpdate = np.row_stack((newPoint, pathUpdate))

        return pathUpdate

    def removePathBehindCar(self, pathLengthFixed: FloatArray) -> FloatArray:
        """
        Remove part of the path that is behind the car.
        """
        idxStartMpcPath = int(self.costMpcPathStart(pathLengthFixed).argmin())
        pathLengthFixedForward: FloatArray = pathLengthFixed[idxStartMpcPath:]
        return pathLengthFixedForward

    def removePathNotInPredictionHorizon(self, pathLengthFixedForward: FloatArray) -> FloatArray:
        """
        If the path with fixed length is too long, for the needs of MPC, it is
        truncated to the desired length.
        """
        distances = traceDistanceToNext(pathLengthFixedForward)
        cumDist = np.cumsum(distances)
        # the code crashes if cum_dist is smaller than mpc_path_length -->
        # atm mpc_path_length has to be long enough so that doesn't happen
        # TODO: change it so that it is not dependent on mpc_path_length
        maskCumDistanceOverMcpPathLength: BoolArray = cumDist > self.scalars.mpcPathLength
        if len(maskCumDistanceOverMcpPathLength) <= 1:
            return self.previousPaths[-15:]

        firstPointOverDistance = cast(int, maskCumDistanceOverMcpPathLength.argmax())

        # if all the elements in the mask are false then argmax will return 0, we need
        # to detect this case and use the whole path when this happens
        if firstPointOverDistance == 0 and not maskCumDistanceOverMcpPathLength[0]:
            firstPointOverDistance = len(cumDist)
        pathWitLengthForMpc: FloatArray = pathLengthFixedForward[:firstPointOverDistance]
        return pathWitLengthForMpc

    def storePaths(
        self,
        pathUpdate: FloatArray,
        pathIsTrivial: bool,
    ) -> None:
        """
        Store the calculated paths, in case they are need in the next calculation.
        """
        self.pathUpdates = self.pathUpdates[-10:] + [pathUpdate]
        self.pathIsTrivialList = self.pathIsTrivialList[-10:] + [pathIsTrivial]

    def runPathCalculation(self) -> FloatArray:
        """Calculate path."""
        if len(self.input.leftCones) < 3 and len(self.input.rightCones) < 3:
            if len(self.previousPaths) > 0:
                # extract x, y from previously calculated path
                centerAlongMatchConnection = self.previousPaths[-15:]
            else:
                centerAlongMatchConnection = self.calculateTrivialPath()
        elif self.input.globalPath is None:
            (
                sideToUse,
                matchesToOtherSide,
                otherSideCones,
            ) = self.selectSideToUse()

            matchOnOtherSide = otherSideCones[matchesToOtherSide]

            centerAlongMatchConnection = self.calculateCenterlinePointsOfMatches(
                sideToUse, matchesToOtherSide, matchOnOtherSide
            )
        else:
            distance = np.linalg.norm(self.input.positionGlobal - self.input.globalPath, axis=1)

            idxClosestPointToPath = distance.argmin()

            rollValue = -idxClosestPointToPath + len(self.input.globalPath) // 3

            pathRolled = np.roll(self.input.globalPath, rollValue, axis=0)
            distanceRolled = np.roll(distance, rollValue)
            maskDistance = distanceRolled < 30
            pathRolled = pathRolled[maskDistance]
            centerAlongMatchConnection = pathRolled

        pathUpdate = self.overwritePathIfItIsTooFarAway(centerAlongMatchConnection)

        pathWithLengthForMpc = self.createPathForMpcFromPathUpdate(pathUpdate)

        self.storePaths(pathWithLengthForMpc, False)
        self.previousPaths = np.concatenate(
            (self.previousPaths[-15:], pathWithLengthForMpc), axis=0
        )

        return pathWithLengthForMpc
