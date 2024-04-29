"""
Description: Combines the result of the search along the left and right traces
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from src.types_file.types import FloatArray, IntArray
from src.utils.math_utils import (
    angleDifference,
    angleFrom2dVector,
)


def calcFinalConfigsForLeftAndRight(
    leftScores: Optional[FloatArray],
    leftConfigs: Optional[IntArray],
    rightScores: Optional[FloatArray],
    rightConfigs: Optional[IntArray],
    cones: FloatArray,
) -> tuple[IntArray, IntArray]:
    """
    Calculates the final configurations for the left and right sides based on the input scores,
    configurations, cones, car position, and car direction.

    Args:
        leftScores (Optional[FloatArray]): The scores for the left side configurations.
        leftConfigs (Optional[IntArray]): The configurations for the left side.
        rightScores (Optional[FloatArray]): The scores for the right side configurations.
        rightConfigs (Optional[IntArray]): The configurations for the right side.
        cones (FloatArray): The array of cones.
        carPos (FloatArray): The position of the car.
        carDir (FloatArray): The direction of the car.

    Returns:
        tuple[IntArray, IntArray]: A tuple containing the final configurations for the
                                   left and right sides.

    """

    nNonNone = sum(x is not None for x in (leftScores, rightScores))

    # if both sides are None, we have no valid configuration
    emptyConfig = np.zeros(0, dtype=int)
    emptyResult = (emptyConfig, emptyConfig)

    if nNonNone == 0:
        return emptyResult

    if nNonNone == 1:
        # only one side has a valid configuration
        return calcFinalConfigsWhenOnlyOneSideHasConfigs(
            leftConfigs,
            rightConfigs,
        )
    if leftConfigs is None or rightConfigs is None:
        return emptyResult
    # both sides have valid configurations
    # we need to pick the best one for each side
    return calcFinalConfigsForBothAvailable(
        leftConfigs,
        rightConfigs,
        cones
    )


def calcFinalConfigsWhenOnlyOneSideHasConfigs(
    leftConfigs: Optional[IntArray],
    rightConfigs: Optional[IntArray],
) -> tuple[IntArray, IntArray]:
    """
    Calculate the final configurations when only one side has configurations.

    Args:
        leftConfigs (Optional[IntArray]): The configurations for the left side.
        rightConfigs (Optional[IntArray]): The configurations for the right side.

    Returns:
        tuple[IntArray, IntArray]: A tuple containing the final configurations for the 
                                   left side and the right side.
    """
    emptyConfig = np.zeros(0, dtype=int)

    leftConfigsIsNone = leftConfigs is None
    rightConfigsIsNone = rightConfigs is None

    assert leftConfigsIsNone != rightConfigsIsNone

    if leftConfigs is None and rightConfigs is not None:
        leftConfig = emptyConfig
        rightConfig = rightConfigs[0]
        rightConfig = rightConfig[rightConfig != -1]
    elif rightConfigs is None and leftConfigs is not None:
        leftConfig = leftConfigs[0]
        leftConfig = leftConfig[leftConfig != -1]
        rightConfig = emptyConfig
    else:
        raise ValueError("Should not happen")

    return leftConfig, rightConfig


def calcFinalConfigsForBothAvailable(
    leftConfigs: IntArray,
    rightConfigs: IntArray,
    cones: FloatArray,
) -> tuple[IntArray, IntArray]:
    """
    Calculates the final configurations for both available sides based on the given
    left and right configurations and cone positions.

    Args:
        leftConfigs (IntArray): An array of left configurations.
        rightConfigs (IntArray): An array of right configurations.
        cones (FloatArray): An array of cone positions.

    Returns:
        tuple[IntArray, IntArray]: A tuple containing the final left and right configurations.

    """
    # we need to pick the best one for each side

    leftConfig = leftConfigs[0]
    leftConfig = leftConfig[leftConfig != -1]

    rightConfig = rightConfigs[0]
    rightConfig = rightConfig[rightConfig != -1]

    leftConfig, rightConfig = handleSameConeInBothConfigs(
        cones,
        leftConfig,
        rightConfig,
    )
    assert leftConfig is not None
    assert rightConfig is not None
    return (leftConfig, rightConfig)


def handleSameConeInBothConfigs(
    cones: FloatArray,
    leftConfig: IntArray,
    rightConfig: IntArray,
) -> tuple[Optional[IntArray], Optional[IntArray]]:
    """
    Handles the case where the same cone is present in both leftConfig and rightConfig.

    Args:
        cones (FloatArray): Array of cone values.
        leftConfig (IntArray): Array representing the left configuration.
        rightConfig (IntArray): Array representing the right configuration.

    Returns:
        tuple[Optional[IntArray], Optional[IntArray]]: A tuple containing the updated
            leftConfig and rightConfig.
            If there are no common cones, the original leftConfig and rightConfig are returned.
    """
    (
        sameConeIntersection,
        leftIntersectionIdxs,
        rightIntersectionIdxs,
    ) = np.intersect1d(leftConfig, rightConfig, return_indices=True)
    if len(sameConeIntersection) == 0:
        return leftConfig, rightConfig

    leftIntersectionIndex = min(leftIntersectionIdxs)  # first index of common cone in leftConfig
    rightIntersectionIndex = min(rightIntersectionIdxs)  # first index of common cone in rightConfig

    # if both sides have the same FIRST common cone, then we try to find the side
    # to which the cone probably belongs
    (leftStopIdx, rightStopIdx,) = calcNewLengthForConfigsForSameConeIntersection(
        cones,
        leftConfig,
        rightConfig,
        leftIntersectionIndex,
        rightIntersectionIndex,
    )

    leftConfig = leftConfig[:leftStopIdx]
    rightConfig = rightConfig[:rightStopIdx]

    return leftConfig, rightConfig


def calcNewLengthForConfigsForSameConeIntersection(
    cones: FloatArray,
    leftConfig: IntArray,
    rightConfig: IntArray,
    leftIntersectionIndex: int,
    rightIntersectionIndex: int,
) -> tuple[Optional[int], Optional[int]]:
    """
    Calculates the new length for configurations with the same cone intersection.

    Args:
        cones (FloatArray): Array of cone coordinates.
        leftConfig (IntArray): Array representing the left configuration.
        rightConfig (IntArray): Array representing the right configuration.
        leftIntersectionIndex (int): Index of the left intersection cone.
        rightIntersectionIndex (int): Index of the right intersection cone.

    Returns:
        tuple[Optional[int], Optional[int]]: A tuple containing the indices to stop the
                                             left and right configurations.

    """
    conesXY = cones[:, :2]
    if leftIntersectionIndex > 0 and rightIntersectionIndex > 0:

        distIntersectionToPrevLeft = np.linalg.norm(
            conesXY[leftConfig[leftIntersectionIndex]]
              - conesXY[leftConfig[leftIntersectionIndex - 1]])
        distIntersectionToPrevRight = np.linalg.norm(
            conesXY[leftConfig[leftIntersectionIndex]]
              - conesXY[rightConfig[rightIntersectionIndex - 1]])

        leftDistIsVeryLow = distIntersectionToPrevLeft < 3.0
        rightDistIsVeryLow = distIntersectionToPrevRight < 3.0

        if ((leftDistIsVeryLow or rightDistIsVeryLow)
            and not (leftDistIsVeryLow and rightDistIsVeryLow)):
            if leftDistIsVeryLow:
                leftStopIdx = len(leftConfig)
                rightStopIdx = rightIntersectionIndex
            else:
                leftStopIdx = leftIntersectionIndex
                rightStopIdx = len(rightConfig)
        else:
            leftStopIdx = None
            rightStopIdx = None
    else:
        leftStopIdx = None
        rightStopIdx = None

    if (
        leftConfig[leftIntersectionIndex] == rightConfig[rightIntersectionIndex]
        and leftIntersectionIndex in range(1, len(leftConfig) - 1)  # not first or last
        and rightIntersectionIndex in range(1, len(rightConfig) - 1)
    ):
        leftStopIdx, rightStopIdx = intersectionInMiddle(
            leftConfig, rightConfig, leftIntersectionIndex, rightIntersectionIndex, cones)
    elif leftStopIdx is None and rightStopIdx is None:
        # if the intersection is the last cone in the config, we assume that this is an
        # error because the configuration could not continue, so we only remove it from that side

        #Left and right intersection is at end
        if (leftIntersectionIndex == len(leftConfig) - 1
            and rightIntersectionIndex == len(rightConfig) - 1):
            leftStopIdx = len(leftConfig) - 1
            rightStopIdx = len(rightConfig) - 1
        #Left intersection is at end
        elif leftIntersectionIndex == len(leftConfig) - 1:
            rightStopIdx = len(rightConfig)
            leftStopIdx = leftIntersectionIndex
        #Right intersection is at end
        elif rightIntersectionIndex == len(rightConfig) - 1:
            leftStopIdx = len(leftConfig)
            rightStopIdx = rightIntersectionIndex
        else:
            leftStopIdx = leftIntersectionIndex
            rightStopIdx = rightIntersectionIndex
    return leftStopIdx, rightStopIdx

def intersectionInMiddle(
        leftConfig: IntArray,
        rightConfig: IntArray,
        leftIntersectionIndex: int,
        rightIntersectionIndex: int,
        cones: FloatArray,
) -> tuple[int, int]:
    """
    Determines the stop indices for the left and right configurations based on the
    intersection position.

    Args:
        leftConfig (IntArray): The left configuration of cones.
        rightConfig (IntArray): The right configuration of cones.
        leftIntersectionIndex (int): The index of the intersection point in the left configuration.
        rightIntersectionIndex (int): The index of the intersection point in the 
                                      right configuration.
        cones (FloatArray): The array of cone positions.
        leftStopIdx (int): The stop index for the left configuration.
        rightStopIdx (int): The stop index for the right configuration.

    Returns:
        tuple[int, int]: A tuple containing the stop indices for the left and right configurations.
    """

    # intersection happens in the middle of the config
    signAngleLeft = np.sign(
        calcAngleChangeAtPosition(cones[:, :2], leftConfig, leftIntersectionIndex))
    signAngleRight = np.sign(
        calcAngleChangeAtPosition(cones[:, :2], rightConfig, rightIntersectionIndex))

    nConesDiff = abs(len(leftConfig) - len(rightConfig))

    if signAngleLeft == signAngleRight:
        if signAngleLeft == 1:
            # this is a left corner, prefer the left
            leftStopIdx = len(leftConfig)
            rightStopIdx = rightIntersectionIndex
            return leftStopIdx, rightStopIdx

        # this is a right corner, prefer the right
        leftStopIdx = leftIntersectionIndex
        rightStopIdx = len(rightConfig)
        return leftStopIdx, rightStopIdx
    if nConesDiff > 2:
        # if the difference in the number of cones is greater than 2, we assume the longer config
        # is the correct one
        if len(leftConfig) > len(rightConfig):
            leftStopIdx = len(leftConfig)
            rightStopIdx = rightIntersectionIndex
            return leftStopIdx, rightStopIdx

        leftStopIdx = leftIntersectionIndex
        rightStopIdx = len(rightConfig)
        return leftStopIdx, rightStopIdx

    leftStopIdx = leftIntersectionIndex
    rightStopIdx = rightIntersectionIndex
    return leftStopIdx, rightStopIdx

def calcAngleChangeAtPosition(
    cones: FloatArray,
    config: IntArray,
    positionInConfig: int,
) -> float:
    """
    Calculates the angle change at a given position in the configuration.

    Args:
        cones (FloatArray): An array of cones.
        config (IntArray): An array of cone configurations.
        positionInConfig (int): The position in the configuration.

    Returns:
        float: The angle change at the given position in the configuration.
    """
    previousCone, intersectionCone, nextCone = cones[
        config[positionInConfig - 1 : positionInConfig + 2], :2
    ]

    intersectionToNext = nextCone - intersectionCone
    intersectionToPrev = previousCone - intersectionCone

    angleIntersectionToNext = angleFrom2dVector(intersectionToNext)
    angleIntersectionToPrev = angleFrom2dVector(intersectionToPrev)

    angle = angleDifference(angleIntersectionToNext, angleIntersectionToPrev)

    return float(angle)
