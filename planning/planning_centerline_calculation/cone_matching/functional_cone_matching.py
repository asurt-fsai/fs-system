#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Description: Match cones from left and right trace to facilitate
more stable path calculation
"""

from __future__ import annotations

from typing import Literal, Tuple, cast

import numpy as np
from numpy.typing import NDArray

from icecream import ic  # pylint: disable=unused-import

from .match_directions import (
    calculateMatchSearchDirection,
)
#from ..types import NDArray[np.bool_], NDArray[np.float_], NDArray[np.int_]
from utils.cone_types import ConeTypes
from utils.math_utils import (
    angleFrom2dVector,
    myCdistSqEuclidean,
    myNjit,
    rotate,
    traceAnglesBetween,
    vecAngleBetween,
)

ic = lambda x: x  # pylint: disable=invalid-name


@myNjit
def conesInRangeAndPovMask(
    cones: NDArray[np.float_],
    searchDirections: NDArray[np.float_],
    searchRange: float,
    searchAngle: float,
    otherSideCones: NDArray[np.float_],
) -> NDArray[np.bool_]:
    """
    Calculates the indices of the visible cones according to the car position

    Returns:
        The indices of the visible cones
    """
    searchRangeSquared = searchRange * searchRange

    # # (M, 2), (N, 2) -> (M,N)
    distFromConesToOtherSideSquared: NDArray[np.float_] = myCdistSqEuclidean(
        otherSideCones, cones
    )

    # (M, N)
    distMask: NDArray[np.bool_] = distFromConesToOtherSideSquared < searchRangeSquared

    # (M, 1, 2) - (N, 2) -> (M, N, 2)
    vecFromConesToOtherSide = np.expand_dims(otherSideCones, axis=1) - cones

    # (N, 2) -> (M, N, 2)
    searchDirectionsBroadcasted: NDArray[np.float_] = np.broadcast_to(
        searchDirections, vecFromConesToOtherSide.shape
    ).copy()  # copy is needed in numba, otherwise a reshape error occurs

    # (M, N, 2), (M, N, 2) -> (M, N)
    anglesToCar = vecAngleBetween(
        searchDirectionsBroadcasted, vecFromConesToOtherSide
    )
    # (M, N)
    maskAngles = np.logical_and(
        -searchAngle / 2 < anglesToCar, anglesToCar < searchAngle / 2
    )

    visibleConesMask = np.logical_and(distMask, maskAngles)

    returnValue: NDArray[np.bool_] = visibleConesMask
    return returnValue


def findBooleanMaskOfAllPotentialMatches(
    starPoints: NDArray[np.float_],
    directions: NDArray[np.float_],
    otherSideCones: NDArray[np.float_],
    otherSideDirections: NDArray[np.float_],
    majorRadius: float,
    minorRadius: float,
    maxSearchAngle: float,
) -> NDArray[np.bool_]:
    """
    Calculate a (M,N) boolean mask that indicates for each cone in the cones array
    if a cone on the other side can be match or not.
    """

    returnValue = np.zeros((len(starPoints), len(otherSideCones)), dtype=bool)
    if len(starPoints) == 0 or len(otherSideCones) == 0:
        return returnValue

    # return mask_all

    # (2,)
    radiiSquare = np.array([majorRadius, minorRadius]) ** 2

    # (M,)
    angles = angleFrom2dVector(directions)
    # (M, N, 2)                       (N, 2)             (M, 1, 2)
    fromStartPointsToOtherSide = otherSideCones - starPoints[:, None]

    startPointDirectionOtherSideDirectionAngleDiff = vecAngleBetween(
        directions[:, None], otherSideDirections
    )

    for i, (
        startPointToOtherSideCones,
        angle,
        angleDiffStartDirectionOtherDirection,
    ) in enumerate(
        zip(
            fromStartPointsToOtherSide,
            angles,
            startPointDirectionOtherSideDirectionAngleDiff,
        )
    ):
        # (N, 2)
        rotatedStartPointToOtherSide = rotate(
            startPointToOtherSideCones, -angle
        )
        # (N,)

        s = (rotatedStartPointToOtherSide**2 / radiiSquare).sum(axis=1)
        returnValue[i] = s < 1

        angleOfRotatedStartPointToOtherSide = angleFrom2dVector(
            rotatedStartPointToOtherSide
        )

        maskAngleIsOverThreshold = (
            np.abs(angleOfRotatedStartPointToOtherSide / 2) > maxSearchAngle
        )

        maskDirectionDiffOverThreshold = (
            angleDiffStartDirectionOtherDirection < np.pi / 2
        )

        returnValue[i, maskAngleIsOverThreshold] = False
        returnValue[i, maskDirectionDiffOverThreshold] = False

    for i, (maskConeToCandidates, distanceToOtherSide) in enumerate(
        zip(returnValue, np.linalg.norm(fromStartPointsToOtherSide, axis=-1))
    ):
        distanceToOtherSide[~maskConeToCandidates] = np.inf
        idxsCandidatesSorted = np.argsort(distanceToOtherSide)[:2]
        mask_idx_candidate_is_valid = np.isfinite(
            distanceToOtherSide[idxsCandidatesSorted]
        )
        idxsCandidatesSorted = idxsCandidatesSorted[mask_idx_candidate_is_valid]

        newMask = np.zeros_like(maskConeToCandidates)
        newMask[idxsCandidatesSorted] = True
        returnValue[i] = newMask

    return returnValue


def selectBestMatchCandidate(
    matchableCones: NDArray[np.float_],
    matchDirections: NDArray[np.float_],
    matchBooleanMask: NDArray[np.bool_],
    otherSideCones: NDArray[np.float_],
    matchesShouldBeMonotonic: bool,
) -> NDArray[np.int_]:
    """
    For each cone select a matching cone from the other side. If a cone has no potential
    match, it is marked with -1.
    """

    if len(otherSideCones) == 0:
        return np.full(len(matchableCones), -1, dtype=int)

    matchedIndexForEachCone: NDArray[np.int_] = myCdistSqEuclidean(
        matchableCones, otherSideCones
    ).argmin(axis=1)

    if matchesShouldBeMonotonic:
        # constraint matches to be monotonic
        currentMaxValue = matchedIndexForEachCone[0]
        for i in range(1, len(matchedIndexForEachCone)):
            currentMaxValue = max(currentMaxValue, matchedIndexForEachCone[i])
            if matchedIndexForEachCone[i] != currentMaxValue:
                matchedIndexForEachCone[i] = -1
            else:
                matchedIndexForEachCone[i] = currentMaxValue

    matchedIndexForEachCone[~matchBooleanMask.any(axis=1)] = -1
    return matchedIndexForEachCone


def calculatePositionsOfVirtualCones(
    cones: NDArray[np.float_],
    indicesOfUnmatchedCones: NDArray[np.int_],
    searchDirections: NDArray[np.float_],
    minTrackWidth: float,
) -> NDArray[np.float_]:
    """
    Calculate the positions of the virtual cones given the unmatched cones and the
    direction of the match search.
    """

    returnValue: NDArray[np.float_] = (
        cones[indicesOfUnmatchedCones]
        + searchDirections[indicesOfUnmatchedCones] * minTrackWidth
    )
    return returnValue


def insertVirtualConesToExisting(
    otherSideCones: NDArray[np.float_],
    otherSideVirtualCones: NDArray[np.float_],
    carPosition: NDArray[np.float_],
) -> NDArray[np.float_]:
    """
    Combine the virtual with the real cones into a single array.
    """
    # print(locals())
    existingCones, conesToInsert = (
        (otherSideCones, otherSideVirtualCones)
        if len(otherSideCones) > len(otherSideVirtualCones)
        else (otherSideVirtualCones, otherSideCones)
    )
    existingCones = existingCones.copy()
    conesToInsert = conesToInsert.copy()

    orderToInsert = (
        myCdistSqEuclidean(conesToInsert, existingCones).min(axis=1).argsort()
    )
    conesToInsert = conesToInsert[orderToInsert]

    for coneToInsert in conesToInsert:
        distanceToExistingCones = np.linalg.norm(
            existingCones - coneToInsert, axis=1
        )
        indicesSortedByDistances = distanceToExistingCones.argsort()

        if len(indicesSortedByDistances) == 1:
            indexToInsert = calculateInsertIndexForOneCone(
                carPosition, existingCones, coneToInsert
            )
        else:
            closestIndex, secondClosestIndex = indicesSortedByDistances[:2]

            if np.abs(closestIndex - secondClosestIndex) != 1:
                continue

            virtualToClosest = existingCones[closestIndex] - coneToInsert
            virtualToSecondClosest = (
                existingCones[secondClosestIndex] - coneToInsert
            )
            angleBetweenVirtualConesAndClosestTwo = vecAngleBetween(
                virtualToClosest, virtualToSecondClosest
            )

            coneIsBetweenClosestTwo = cast(
                bool, angleBetweenVirtualConesAndClosestTwo > np.pi / 2
            )

            indexToInsert = calculateInsertIndexOfNewCone(
                closestIndex,
                secondClosestIndex,
                coneIsBetweenClosestTwo,
            )

        existingCones: NDArray[np.float_] = np.insert(  # type: ignore
            existingCones,
            indexToInsert,
            coneToInsert,
            axis=0,
        )


    angles = traceAnglesBetween(existingCones)
    # print(np.rad2deg(angles))
    mask_low_angles = angles < np.deg2rad(85)
    mask_low_angles = np.concatenate([[False], mask_low_angles, [False]])

    if mask_low_angles.any():
        existingCones = existingCones[:][~mask_low_angles]
    
    return existingCones


def calculateInsertIndexForOneCone(
    carPosition: NDArray[np.float_], finalCones: NDArray[np.float_], virtualCone: NDArray[np.float_]
) -> int:
    """
    Insert a virtual cone into the real cones, when only one real cone is available.
    The position of the virtual cone in the array is based on distance to the car.
    """
    distanceToCarOtherCone: float = np.linalg.norm(
        virtualCone - carPosition,
    )  # type: ignore
    distanceToCarExistingCone: float = np.linalg.norm(
        finalCones[0] - carPosition,
    )  # type: ignore

    if distanceToCarOtherCone < distanceToCarExistingCone:
        indexToInsert = 0
    else:
        indexToInsert = 1
    return indexToInsert


def calculateInsertIndexOfNewCone(
    closestIndex: int,
    secondClosestIndex: int,
    coneIsBetweenClosestTwo: bool,
) -> int:
    """
    Decide the index of the new cone to insert. It is based on the distance to the
    two closest cones and the angle that is formed between them.
    """

    if coneIsBetweenClosestTwo:
        return min(closestIndex, secondClosestIndex) + 1
    else:
        if closestIndex < secondClosestIndex:
            return closestIndex
        elif closestIndex > secondClosestIndex:
            return closestIndex + 1
        else:
            raise ValueError("Unreachable code")


def combineAndSortVirtualWithReal(
    otherSideCones: NDArray[np.float_],
    otherSideVirtualCones: NDArray[np.float_],
    carPos: NDArray[np.float_],
) -> Tuple[NDArray[np.float_], NDArray[np.bool_]]: #removed history
    """
    Combine the existing cones with the newly calculated cones into a single array.
    """

    if len(otherSideCones) == 0:
        return (
            otherSideVirtualCones,
            np.ones(len(otherSideVirtualCones), dtype=bool),
            [],
        )

    if len(otherSideVirtualCones) == 0:
        return otherSideCones, np.zeros(len(otherSideCones), dtype=bool), []

    sortedCombinedCones = insertVirtualConesToExisting(
        otherSideCones, otherSideVirtualCones, carPos
    )

    # cones that have a distance larger than epsilon to the existing cones are virtual
    distanceOfFinalConesToExisting = myCdistSqEuclidean(
        sortedCombinedCones, otherSideCones
    )
    epsilon = 1e-2
    virtualMask: NDArray[np.bool_] = distanceOfFinalConesToExisting > epsilon**2
    maskIsVirtual: NDArray[np.bool_] = np.all(virtualMask, axis=1)

    return sortedCombinedCones, maskIsVirtual


def calculateMatchForSide(
    cones: NDArray[np.float_],
    coneType: ConeTypes,
    otherSideCones: NDArray[np.float_],
    majorRadius: float,
    minorRadius: float,
    maxSearchAngle: float,
    matchesShouldBeMonotonic: bool,
) -> Tuple[NDArray[np.float_], NDArray[np.int_], NDArray[np.float_]]:
    """
    Find a match for each cone from one side to the other.
    """
    matchableCones = cones[:]
    if len(matchableCones) > 1:
        searchDirections = calculateMatchSearchDirection(matchableCones, coneType)

        if len(otherSideCones) > 1:
            otherSideSearchDirections = calculateMatchSearchDirection(
                otherSideCones,
                ConeTypes.LEFT if coneType == ConeTypes.RIGHT else ConeTypes.RIGHT,
            )
        else:
            otherSideSearchDirections = np.zeros((0, 2), dtype=float)
        usToOthersMatchConesMask = findBooleanMaskOfAllPotentialMatches(
            matchableCones,
            searchDirections,
            otherSideCones,
            otherSideSearchDirections,
            majorRadius,
            minorRadius,
            maxSearchAngle,
        )

        matchesForEachSelectableCone = selectBestMatchCandidate(
            matchableCones,
            searchDirections,
            usToOthersMatchConesMask,
            otherSideCones,
            matchesShouldBeMonotonic,
        )
    else:
        matchesForEachSelectableCone = (
            np.zeros((len(matchableCones),), dtype=np.int32) - 1
        )
        searchDirections = np.zeros((0, 2))

    return matchableCones, matchesForEachSelectableCone, searchDirections


def calculateConesForOtherSide(
    cones: NDArray[np.float_],
    coneType: ConeTypes,
    majorRadius: float,
    minorRadius: float,
    maxSearchAngle: float,
    otherSideCones: NDArray[np.float_],
    minTrackWidth: float,
    carPos: NDArray[np.float_],
    matchesShouldBeMonotonic: bool,
) -> Tuple[NDArray[np.float_], NDArray[np.bool_]]:
    """
    Calculate the virtual cones for the other side.
    """
    (
        matchableCones,
        matchesForEachSelectableCone,
        searchDirections,
    ) = calculateMatchForSide(
        cones,
        coneType,
        otherSideCones,
        majorRadius,
        minorRadius,
        maxSearchAngle,
        matchesShouldBeMonotonic,
    )
    maskConeHasMatch = matchesForEachSelectableCone != -1
    # indices_to_keep = np.where(mask_cone_has_match)[0]
    indicesNoMatch = np.where(~maskConeHasMatch)[0]

    positionsOfVirtualCones = calculatePositionsOfVirtualCones(
        matchableCones, indicesNoMatch, searchDirections, minTrackWidth
    )

    #removedOtherSideConeType

    # we do not care about the history in prod, only for debugging/visualization
    result = combineAndSortVirtualWithReal(
        otherSideCones,
        positionsOfVirtualCones,
        carPos,
    )
    combinedAndSortedCones = result[0]
    maskIsVirtual = result[1]
    if len(combinedAndSortedCones) < 2:
        combinedAndSortedCones = otherSideCones
        maskIsVirtual = np.zeros(len(otherSideCones), dtype=bool)

    return combinedAndSortedCones, maskIsVirtual


def matchBothSidesWithVirtualCones(
    leftConesWithVirtual: NDArray[np.float_],
    rightConesWithVirtual: NDArray[np.float_],
    majorRadius: float,
    minorRadius: float,
    maxSearchAngle: float,
    matchesShouldBeMonotonic: bool,
) -> Tuple[NDArray[np.int_], NDArray[np.int_]]:
    """
    After virtual cones have been placed for each side, rerun matching algorithm
    to get final matches.
    """

    _, finalMatchingFromLeftToRight, _ = calculateMatchForSide(
        leftConesWithVirtual,
        ConeTypes.LEFT,
        rightConesWithVirtual,
        majorRadius,
        minorRadius,
        maxSearchAngle,
        matchesShouldBeMonotonic,
    )

    _, finalMatchingFromRightToLeft, _ = calculateMatchForSide(
        rightConesWithVirtual,
        ConeTypes.RIGHT,
        leftConesWithVirtual,
        majorRadius,
        minorRadius,
        maxSearchAngle,
        matchesShouldBeMonotonic,
    )

    return finalMatchingFromLeftToRight, finalMatchingFromRightToLeft


def calculateVirtualConesForBothSides(
    leftCones: NDArray[np.float_],
    rightCones: NDArray[np.float_],
    carPosition: NDArray[np.float_],
    minTrackWidth: float,
    majorRadius: float,
    minorRadius: float,
    maxSearchAngle: float,
    matchesShouldBeMonotonic: bool = True,
) -> Tuple[
    Tuple[NDArray[np.float_], NDArray[np.bool_], NDArray[np.int_]],
    Tuple[NDArray[np.float_], NDArray[np.bool_], NDArray[np.int_]],
]:
    """
    The main function of the module. It applies all the steps to return two results
    containing the new cones including a virtual ones, a boolean mask indicating for
    each cone if it is a virtual cone or not and an integer mask indicating for each
    cone the index of the match on the other side.
    """
    # if len(left_cones) > 20 or len(right_cones) > 20:
    #     print(locals())
    emptyBoolArr: NDArray[np.bool_] = np.zeros(0, dtype=np.bool_)
    emptyIntArr: NDArray[np.int_] = np.zeros(0, dtype=np.int_)
    emptyConeArray: NDArray[np.float_] = np.zeros((0, 2), dtype=np.float_)

    dummyResult = emptyConeArray, emptyBoolArr, emptyIntArr

    ic("calculate_virtual_cones_for_both_sides: start")
    if len(leftCones) < 2 and len(rightCones) < 2:
        leftResult = dummyResult
        rightResult = dummyResult
        return leftResult, rightResult

    minLen = min(len(leftCones), len(rightCones))
    maxLen = max(len(leftCones), len(rightCones))
    discardOneSide = minLen == 0 or ((maxLen / minLen) > 2)
    if discardOneSide:
        if len(leftCones) < len(rightCones):
            leftCones = emptyConeArray
        else:
            rightCones = emptyConeArray

    ic("calculate_virtual_cones_for_both_sides: left match right")
    rightMaskIsVirtual: NDArray[np.bool_]
    rightConesWithVirtual, rightMaskIsVirtual = (
        calculateConesForOtherSide(
            leftCones,
            ConeTypes.LEFT,
            majorRadius,
            minorRadius,
            maxSearchAngle,
            rightCones,
            minTrackWidth,
            carPosition,
            matchesShouldBeMonotonic,
        )
        if len(leftCones) >= 2
        else (
            rightCones,
            np.zeros(len(rightCones), dtype=bool),
        )
    )

    ic("calculate_virtual_cones_for_both_sides: right match left")
    leftMaskIsVirtual: NDArray[np.bool_]
    leftConesWithVirtual, leftMaskIsVirtual = (
        calculateConesForOtherSide(
            rightCones,
            ConeTypes.RIGHT,
            majorRadius,
            minorRadius,
            maxSearchAngle,
            leftCones,
            minTrackWidth,
            carPosition,
            matchesShouldBeMonotonic,
        )
        if len(rightCones) >= 2
        else (
            leftCones,
            np.zeros(len(leftCones), dtype=bool),
        )
    )

    ic("calculate_virtual_cones_for_both_sides: match left and right w/ virtual")
    leftToRightMatches, rightToLeftMatches = matchBothSidesWithVirtualCones(
        leftConesWithVirtual,
        rightConesWithVirtual,
        majorRadius,
        minorRadius,
        maxSearchAngle,
        matchesShouldBeMonotonic,
    )

    leftResult = (
        leftConesWithVirtual,
        leftMaskIsVirtual,
        leftToRightMatches,
    )
    ic("match left and right w/ virtual")
    rightResult = (
        rightConesWithVirtual,
        rightMaskIsVirtual,
        rightToLeftMatches,
    )
    return leftResult, rightResult
