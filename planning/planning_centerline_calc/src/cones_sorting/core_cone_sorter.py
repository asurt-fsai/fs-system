"""
Description: This module provides functionality for sorting a trace of cones into aplausible track
"""

from __future__ import annotations
from typing import Optional, Tuple, cast

import numpy as np

from src.utils.cone_types import ConeTypes, invertConeType
from src.types_file.types import FloatArray, IntArray, BoolArray
from src.utils.math_utils import (
    angleFrom2dVector,
    rotate,
    pointsInsideEllipse,
    unit2dVectorFromAngle,
    vecAngleBetween,
    myCdistSqEuclidean,
)
from src.cones_sorting.combine_traces import calcFinalConfigsForLeftAndRight
from src.cones_sorting.adjecency_matrix import AdjacencyMatrix
from src.cones_sorting.end_configurations import findAllEndConfigurations, NoPathError
from src.cones_sorting.cost_function import costConfigurations
from src.cone_matching.functional_cone_matching import combineAndSortVirtualWithReal


def flattenConesByTypeArray(conesByType: list[FloatArray]) -> FloatArray:
    """Ravel the conesByType array"""

    if isinstance(conesByType, np.ndarray) and conesByType.ndim == 2 and conesByType.shape[1] == 3:
        return conesByType

    nAllCones = sum(map(len, conesByType))

    # (x, y, color)
    out = np.empty((nAllCones, 3))
    nStart = 0
    for coneType in ConeTypes:
        nCones = len(conesByType[coneType])
        out[nStart : nStart + nCones, :2] = conesByType[coneType].reshape(-1, 2)
        out[nStart : nStart + nCones, 2] = coneType
        nStart += nCones

    return out


class ConeSorter:
    """
    Wraps the cone sorting functionality into a class
    """

    def __init__(
        self,
        maxNNeighbors: int,
        maxDist: float,
        maxDistToFirst: float,
        maxLength: int,
        thresholdDirectionalAngle: float,
        thresholdAbsoluteAngle: float,
    ) -> None:
        """
        Constructor for ConeSorter class
        """
        self.maxNNeighbors = maxNNeighbors
        self.maxDist = maxDist
        self.maxDistToFirst = maxDistToFirst
        self.maxLength = maxLength
        self.thresholdDirectionalAngle = thresholdDirectionalAngle
        self.thresholdAbsoluteAngle = thresholdAbsoluteAngle

    def selectFirstKStartingCones(
        self,
        carPosition: FloatArray,
        carDirection: np.float_,
        cones: FloatArray,
        coneType: ConeTypes,
    ) -> Optional[IntArray]:
        """
        Return the index of the starting cones. Pick the cone that is closest in front
        of the car and the cone that is closest behind the car.
        """
        index1 = self.selectStartingCone(carPosition, carDirection, cones, coneType)

        if index1 is None:
            return None

        # The mask should not be selected if the absolute angle between the distance
        #  of the cone to the car and the car is less than pi/2
        maskShouldBeNotSelected = (
            np.abs(
                vecAngleBetween(
                    cones[:, :2] - carPosition, unit2dVectorFromAngle(np.array(carDirection))
                )
            )
            < np.pi / 2
        )
        idxsToSkip = np.where(maskShouldBeNotSelected)[0]
        if index1 not in idxsToSkip:
            idxsToSkip = np.concatenate([idxsToSkip, np.array([index1])])

        # get the cone behind the car
        index2 = self.selectStartingCone(
            carPosition, carDirection, cones, coneType, indexToSkip=idxsToSkip
        )

        if index2 is None:
            return np.array([index1], dtype=np.int_)

        # Switch if the angle between the first cone and the car is larger than
        #  the second cone and the car
        if vecAngleBetween(
            cones[index1, :2] - cones[index2, :2], unit2dVectorFromAngle(np.array(carDirection))
        ) > vecAngleBetween(
            cones[index2, :2] - cones[index1, :2], unit2dVectorFromAngle(np.array(carDirection))
        ):
            index1, index2 = index2, index1

        # If the distance between the first cone and the car is larger than the MaxDist*1.1
        #  or the distance is less than 1.4 than return only the first cone
        if (
            np.linalg.norm(cones[index1, :2] - cones[index2, :2]) > self.maxDist * 1.1
            or np.linalg.norm(cones[index1, :2] - cones[index2, :2]) < 1.4
        ):
            return np.array([index1], dtype=np.int_)

        twoCones: IntArray = np.array([index2, index1], dtype=np.int_)

        # find the third cone
        index3 = self.selectStartingCone(
            carPosition, carDirection, cones, coneType, indexToSkip=twoCones
        )
        # Combine the position of the two cones with the third cone
        newCones, *_ = combineAndSortVirtualWithReal(
            cones[twoCones, :2], cones[index3, :2][None], carPosition
        )

        last, middle, first = myCdistSqEuclidean(newCones, cones[:, :2]).argmin(axis=1)

        # If the angle between the second cone to the car and the car is less than pi/2
        #  or there is no index3
        # or if the third cone is close enough to the first cone
        if (
            vecAngleBetween(
                cones[index2, :2] - carPosition, unit2dVectorFromAngle(np.array(carDirection))
            )
            > np.pi / 2
            or index3 is None
            or (
                np.linalg.norm(cones[index3, :2] - cones[twoCones, :2], axis=1).min()
                > self.maxDist * 1.1
            )
        ):
            return twoCones

        # If angle between the middle cone to the last cones
        #  and the middle to the first less than pi/1.5, return 2 cones
        if (
            vecAngleBetween(
                cones[last, :2] - cones[middle, :2], cones[first, :2] - cones[middle, :2]
            )
            < np.pi / 1.5
        ):
            return twoCones

        return np.array([last, middle, first], dtype=np.int_)

    def selectStartingCone(
        self,
        carPosition: FloatArray,
        carDirection: np.float_,
        cones: FloatArray,
        coneType: ConeTypes,
        indexToSkip: Optional[IntArray] = None,
    ) -> Optional[int]:
        """
        Return the index of the starting cone
            int: The index of the stating cone
        """
        traceDistance, maskIsValid = self.maskConeCanBeFisrtInConfig(
            carPosition, carDirection, cones, coneType
        )
        if indexToSkip is not None:
            maskIsValid[indexToSkip] = False

        traceDistanceCopy = traceDistance.copy()
        traceDistanceCopy[~maskIsValid] = np.inf

        if np.any(maskIsValid) > 0:
            sortedIdx = np.argsort(traceDistanceCopy)  # Sort the cones by distance from car
            startIdx = None
            for idx in sortedIdx:
                if indexToSkip is None or idx not in indexToSkip:
                    startIdx = idx
                    break
            if traceDistanceCopy[startIdx] > self.maxDistToFirst:
                startIdx = None
        else:
            startIdx = None

        return startIdx

    def maskConeCanBeFisrtInConfig(
        self,
        carPosition: FloatArray,
        carDirection: np.float_,
        cones: FloatArray,
        coneType: ConeTypes,
    ) -> Tuple[FloatArray, BoolArray]:
        """
        Return a mask of cones that can be the first in a configuration
        """
        conesXY = cones[:, :2]  # remove cone type
        # print(carDirection)

        conesRelative = rotate(
            conesXY - carPosition, -float(carDirection)
        )  # Rotate cones' positions to be relative to the car

        coneRelativeAngles = angleFrom2dVector(conesRelative)

        # cone is in valid sign if the angle sign the same as the valid angle sign
        maskValidSide = np.sign(coneRelativeAngles) == (1 if coneType == ConeTypes.left else -1)
        maskIsValidAngle = np.abs(coneRelativeAngles) < np.pi - np.pi / 5
        maskIsValidAngleMin = np.abs(coneRelativeAngles) > np.pi / 10
        maskIsRightColor = cones[:, 2] == coneType

        maskSide = (maskValidSide * maskIsValidAngle * maskIsValidAngleMin) + maskIsRightColor

        maskIsNotOppositeConeType = cones[:, 2] != invertConeType(coneType)
        # The mask is falid id the points are in the ellipse
        #  and in the correct side and is not an opposite cone type
        maskIsValid = (
            pointsInsideEllipse(
                conesXY,
                carPosition,
                unit2dVectorFromAngle(np.array(carDirection)),
                self.maxDistToFirst * 1.5,
                self.maxDistToFirst / 1.5,
            )
            * maskSide
            * maskIsNotOppositeConeType
        )

        # Return the distances between the car and the cones,
        #  and the mask of cones that can be the first in a configuration
        return np.linalg.norm(conesRelative, axis=-1), maskIsValid

    def calcScoresAndEndConfigurations(
        self,
        trace: FloatArray,
        coneType: ConeTypes,
        startIdx: int,
        vehiclePosition: FloatArray,
        vehicleDirection: np.float_,
        firstKIndicesMustBe: Optional[IntArray] = None,
    ) -> tuple[FloatArray, IntArray, Optional[tuple[IntArray, BoolArray]]]:
        """
        Sorts a set of points such that the sum of the angles between the points is minimal.
        If a point is too far away, from any neighboring points, it is considered an outlier
        and is removed from the ordering
        Args:
            trace: The points to be ordered
            cone_type: The type of cone to be sorted (left/right)
            n_neighbors: The number of neighbors to be considered. For exhaustive
            search set to `len(trace) - 1`
            start_idx: The index of the point to be set first in the ordering.
            vehicle_position: The position of the vehicle
            vehicle_direction: The direction of the vehicle
            first_k_indices_must_be: The indices of the points that must be in the first
        Returns:
            A list of indexes of the points in the optimal ordering, as well as the
            the costs of all end configurations and their corresponding indices
        """
        nNeighbors = min(self.maxNNeighbors, len(trace) - 1)
        adjecencyMatrix, reachableNodes = AdjacencyMatrix(self.maxDist).createAdjacencyMatrix(
            cones=trace,
            nNeighbors=nNeighbors,
            startIdx=startIdx,
            coneType=coneType,
        )
        targetLength = min(reachableNodes.shape[0], self.maxLength)

        if firstKIndicesMustBe is None:
            firstKIndicesMustBe = np.arange(0)

        configurationParameters: Tuple[int, int, float, float] = (
            startIdx,
            targetLength,
            self.thresholdDirectionalAngle,
            self.thresholdAbsoluteAngle,
        )
        vehicleOdometry = (vehiclePosition, vehicleDirection)
        allEndConfigurations = findAllEndConfigurations(
            trace,
            coneType,
            configurationParameters,
            adjecencyMatrix,
            firstKIndicesMustBe,
            vehicleOdometry,
        )
        if len(allEndConfigurations) == 1:
            return (np.array([0.0]).astype(float), allEndConfigurations, None)
        costs = costConfigurations(
            points=trace,
            configurations=allEndConfigurations,
            coneType=coneType,
            vehicleDirection=vehicleDirection,
            returnIndividualCosts=False,
        )
        allEndConfigurations = cast(IntArray, allEndConfigurations[np.argsort(costs)])

        return (cast(FloatArray, costs[np.argsort(costs)]), allEndConfigurations, None)

    def calcConfigurationWithScoresForOneSide(
        self,
        cones: FloatArray,
        coneType: ConeTypes,
        carPos: FloatArray,
        carDir: np.float_,
    ) -> Tuple[Optional[FloatArray], Optional[IntArray]]:
        """
        Args:
            cones: The trace to be sorted.
            coneType: The type of cone to be sorted.
            carPos: The position of the car.
            carDir: The direction towards which the car is facing.
        Returns:
            np.ndarray: The sorted trace, 'len(returnValue) <= len(trace)'
        """
        assert coneType in (ConeTypes.left, ConeTypes.right)

        noResult = None, None

        # if len(cones) < 3:
        #     return noResult

        firstK = self.selectFirstKStartingCones(
            carPos,
            carDir,
            cones,
            coneType,
        )
        if firstK is not None:
            startIdx = firstK[0]
            if len(firstK) > 1:
                firstKIndicesMustBe = firstK.copy()
            else:
                firstKIndicesMustBe = None
        else:
            startIdx = None
            firstKIndicesMustBe = None

        if startIdx is None and firstKIndicesMustBe is None:
            return noResult

        try:
            returnValue = self.calcScoresAndEndConfigurations(
                cones,
                coneType,
                startIdx,
                carPos,
                carDir,
                firstKIndicesMustBe,
            )

        # if no configuration can be found, then return nothing
        except NoPathError:
            return noResult

        return returnValue[:2]

    def sortLeftRight(
        self,
        conesByType: list[FloatArray],
        carPos: FloatArray,
        carDir: np.float_,
    ) -> tuple[FloatArray, FloatArray]:
        """
        Sorts the cones into left and right configurations based on the
        car's position and direction.

        Args:
            conesByType (list[FloatArray]): A list of arrays containing cones grouped by type.
            carPos (FloatArray): The position of the car.
            carDir (FloatArray): The direction of the car.

        Returns:
            tuple[FloatArray, FloatArray]: A tuple containing the sorted
            left and right configurations of cones.
        """
        conesFlat = flattenConesByTypeArray(conesByType)

        (leftScores, leftConfigs) = self.calcConfigurationWithScoresForOneSide(
            conesFlat,
            ConeTypes.left,
            carPos,
            carDir,
        )

        (rightScores, rightConfigs) = self.calcConfigurationWithScoresForOneSide(
            conesFlat,
            ConeTypes.right,
            carPos,
            carDir,
        )

        (leftConfig, rightConfig) = calcFinalConfigsForLeftAndRight(
            leftScores, leftConfigs, rightScores, rightConfigs, conesFlat
        )
        leftConfig = leftConfig[leftConfig != -1]
        rightConfig = rightConfig[rightConfig != -1]

        # remove any placeholder position if they are present
        leftConfig = leftConfig[leftConfig != -1]
        rightConfig = rightConfig[rightConfig != -1]

        leftSorted = conesFlat[leftConfig]
        rightSorted = conesFlat[rightConfig]

        return leftSorted[:, :2], rightSorted[:, :2]
