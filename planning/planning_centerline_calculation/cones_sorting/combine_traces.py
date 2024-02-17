"""
Description: Combines the result of the search along the left and right traces
"""
from __future__ import annotations

from typing import Optional

import numpy as np

# from planning_centerline_calculation.cones_sorting.end_configurations import linesSegmentsIntersectIndicator

from types_file.types import FloatArray, IntArray
from utils.cone_types import ConeTypes
from utils.math_utils import (
    angleDifference,
    angleFrom2dVector,
    myNjit,
)

def calcFinalConfigsForLeftAndRight(
        leftScores: Optional[FloatArray],
        leftConfigs: Optional[IntArray],
        rightScores: Optional[FloatArray],
        rightConfigs: Optional[IntArray],
        cones: FloatArray,
        carPos: FloatArray,
        carDir: FloatArray,
) -> tuple[IntArray, IntArray]:
    leftScoresIsNone = leftScores is None
    rightScoresIsNone = rightScores is None
    assert leftScoresIsNone == rightScoresIsNone

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
    
    # both sides have valid configurations
    # we need to pick the best one for each side

    return calcFinalConfigsForBothAvailable(
        leftScores,
        leftConfigs,
        rightScores,
        rightConfigs,
        cones,
        carPos,
        carDir,
    )
    
def calcFinalConfigsWhenOnlyOneSideHasConfigs(
        leftConfigs: Optional[IntArray],
        rightConfigs: Optional[IntArray],
) -> tuple[IntArray, IntArray]:
    emptyConfig = np.zeros(0, dtype=int)

    leftConfigsIsNone = leftConfigs is None
    rightConfigsIsNone = rightConfigs is None

    assert leftConfigsIsNone != rightConfigsIsNone

    if leftConfigs is None:
        leftConfig = emptyConfig
        rightConfig = rightConfigs[0]
        rightConfig = rightConfig[rightConfig != -1]
    elif rightConfigs is None:
        leftConfig = leftConfigs[0]
        leftConfig = leftConfig[leftConfig != -1]
        rightConfig = emptyConfig
    else:
        raise ValueError("Should not happen")
    
    return leftConfig, rightConfig

def calcFinalConfigsForBothAvailable(
        leftScores: FloatArray,
        leftConfigs: IntArray,
        rightScores: FloatArray,
        rightConfigs: IntArray,
        cones: FloatArray,
        carPosition: FloatArray,
        carDirection: FloatArray,
) -> tuple[IntArray, IntArray]:
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

    return (leftConfig, rightConfig)
    
def handleSameConeInBothConfigs(
        cones: FloatArray,
        leftConfig: IntArray,
        rightConfig: IntArray,
) -> tuple[Optional[IntArray], Optional[IntArray]]:
    (
        sameConeIntersection,
        leftIntersectionIdxs,
        rightIntersectionIdxs,
    ) = np.intersect1d(leftConfig, rightConfig, return_indices=True)
    if len(sameConeIntersection) == 0:
        return leftConfig, rightConfig
    
    leftIntersectionIndex = min(
        leftIntersectionIdxs
    ) # first index of common cone in leftConfig
    rightIntersectionIndex = min(
        rightIntersectionIdxs
    ) # first index of common cone in rightConfig

    # if both sides have the same FIRST common cone, then we try to find the side
    # to which the cone probably belongs
    (
        leftStopIdx,
        rightStopIdx,
    ) = calcNewLengthForConfigsForSameConeIntersection(
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
) -> tuple[int, int]:
    conesXY = cones[:, :2]
    if leftIntersectionIndex > 0 and rightIntersectionIndex > 0:
        prevLeft = leftConfig[leftIntersectionIndex - 1]
        prevRight = rightConfig[rightIntersectionIndex - 1]
        intersectionCone = leftConfig[leftIntersectionIndex]

        distIntersectionToPrevLeft = np.linalg.norm(
            conesXY[intersectionCone] - conesXY[prevLeft]
        )
        distIntersectionToPrevRight = np.linalg.norm(
            conesXY[intersectionCone] - conesXY[prevRight]
        )

        lowDistance = 3.0
        leftDistIsVeryLow = distIntersectionToPrevLeft < lowDistance
        rightDistIsVeryLow = distIntersectionToPrevRight < lowDistance
        anyDistIsVeryLow = leftDistIsVeryLow or rightDistIsVeryLow
        bothDistsAreVeryLow = leftDistIsVeryLow and rightDistIsVeryLow

        if anyDistIsVeryLow and not bothDistsAreVeryLow:
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
        leftStopIdx is None
        and rightStopIdx is None
        and leftConfig[leftIntersectionIndex] 
        == rightConfig[rightIntersectionIndex]
        and leftIntersectionIndex
        in range(1, len(leftConfig) - 1) # not first or last
        and rightIntersectionIndex in range(1, len(rightConfig) - 1)
    ):
        # intersection happens in the middle of the config
        angleLeft = calcAngleChangeAtPosition(
            cones[:, :2], leftConfig, leftIntersectionIndex
        )
        angleRight = calcAngleChangeAtPosition(
            cones[:, :2], rightConfig, rightIntersectionIndex
        )

        signAngleLeft = np.sign(angleLeft)
        signAngleRight = np.sign(angleRight)

        absoluteAngleDiff = abs(abs(angleLeft) - abs(angleRight))

        leftHasThree = len(leftConfig) == 3
        rightHasThree = len(rightConfig) == 3

        nConesDiff = abs(len(leftConfig) - len(rightConfig))

        if signAngleLeft == signAngleRight:
            if signAngleLeft == 1:
                # this is a left corner, prefer the left
                leftStopIdx = len(leftConfig)
                rightStopIdx = rightIntersectionIndex
            else:
                # this is a right corner, prefer the right
                leftStopIdx = leftIntersectionIndex
                rightStopIdx = len(rightConfig)
        elif nConesDiff > 2:
            # if the diffrence in number of cones id greater than 2, we assume the longer config
            # is the correct one
            if len(leftConfig) > len(rightConfig):
                leftStopIdx = len(leftConfig)
                rightStopIdx = rightIntersectionIndex
            else:
                leftStopIdx = leftIntersectionIndex
                rightStopIdx = len(rightConfig)
        else:
            leftStopIdx = leftIntersectionIndex
            rightStopIdx = rightIntersectionIndex
    elif leftStopIdx is None and rightStopIdx is None:
        # if the intersection is the last cone in the config, we assume that this is an
        # error because the configuration could not continue, so we only remove it from that side
        leftIntersectionIsAtEnd = leftIntersectionIndex == len(leftConfig) - 1
        rightIntersectionIsAtEnd = rightIntersectionIndex == len(rightConfig) - 1

        if leftIntersectionIsAtEnd and rightIntersectionIsAtEnd:
            leftStopIdx = len(leftConfig) - 1
            rightStopIdx = len(rightConfig) - 1

        elif leftIntersectionIsAtEnd:
            rightStopIdx = len(rightConfig) 
            leftStopIdx = leftIntersectionIndex
        elif rightIntersectionIsAtEnd:
            leftStopIdx = len(leftConfig)
            rightStopIdx = rightIntersectionIndex
        else:
            leftStopIdx = leftIntersectionIndex
            rightStopIdx = rightIntersectionIndex
    
    return leftStopIdx, rightStopIdx
            

def calcAngleChangeAtPosition(
    cones: FloatArray,
    config: IntArray,
    positionInConfig: int,
) -> float:
    previousCone, intersectionCone, nextCone = cones[
        config[positionInConfig - 1 : positionInConfig + 2], : 2
    ]

    intersectionToNext = nextCone - intersectionCone
    intersectionToPrev = previousCone - intersectionCone

    angleIntersectionToNext = angleFrom2dVector(intersectionToNext)
    angleIntersectionToPrev = angleFrom2dVector(intersectionToPrev)

    angle = angleDifference(angleIntersectionToNext, angleIntersectionToPrev)

    return angle