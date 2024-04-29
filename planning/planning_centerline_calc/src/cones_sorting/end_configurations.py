"""
Description: This File calculates all the possible paths
"""

from typing import TYPE_CHECKING, Optional, Tuple, cast

import numpy as np

from src.types_file.types import BoolArray, FloatArray, GenericArray, IntArray
from src.utils.cone_types import ConeTypes
from src.utils.math_utils import (
    myIn1d,
    pointsInsideEllipse,
    vecAngleBetween,
    unit2dVectorFromAngle,
)
def findAllEndConfigurations(
    points: FloatArray,
    coneType: ConeTypes,
    startIndx: int,
    adjacencyMatrix: IntArray,
    targetLength: int,
    thresholdDirectionalAngle: float,
    thresholdAbsoluteAngle: float,
    firstKIndicesMustBe: IntArray,
    carPosition: FloatArray,
    carDirection: np.float_,
    carSize: float,
    storeAllEndConfigurations: bool,
) -> tuple[IntArray, Optional[tuple[IntArray, BoolArray]]]:
    """
    Finds all the possible paths that include all the reachable nodes from the starting
    Args:
        start_idx: The index of the starting node
        adjacency_matrix: The adjacency matrix indicating which nodes are
        connected to which
        target_length: The length of the path that the search is searching for
    Raises:
        NoPathError: If no path has been found
    Returns:
        A 2d array of configurations (indices) that define a path
    """

    neighborsFlat, borders = adjacecnyMatrixToBordersAndTargets(adjacencyMatrix)
    # print(adjacencyMatrix)
    # print(neighborsFlat)
    # print(borders)
    pointsXY = points[:, :2]

    (
        endConfigurations,
        allConfigurationsAndItsEndConfigurationIndicator,
    ) = implFindAllEndConfigurations(
        pointsXY,
        coneType,
        startIndx,
        neighborsFlat,
        borders,
        targetLength,
        thresholdDirectionalAngle,
        thresholdAbsoluteAngle,
        firstKIndicesMustBe,
        carPosition,
        carDirection,
        carSize,
        storeAllEndConfigurations,
    )

    if len(firstKIndicesMustBe) > 0 and len(endConfigurations) > 0:
        maskKeep = (endConfigurations[:, : len(firstKIndicesMustBe)] == firstKIndicesMustBe).all(
            axis=1
        )
        endConfigurations = endConfigurations[maskKeep]

    # maskLengthIsAtleast3 = (endConfigurations != -1).sum(axis=1) >= 3
    # endConfigurations = endConfigurations[maskLengthIsAtleast3]

    # remove last cone from config if it is of unknown or orange type
    lastConeInEachConfigIdx = (
        np.argmax(endConfigurations == -1, axis=1) - 1
    ) % endConfigurations.shape[1]

    lastConeInEachConfig = endConfigurations[
        np.arange(endConfigurations.shape[0]), lastConeInEachConfigIdx
    ]

    maskLastConeIsNotOfType = points[lastConeInEachConfig, 2] != coneType

    lastConeInEachConfigIdxMasked = lastConeInEachConfigIdx[maskLastConeIsNotOfType]

    endConfigurations[maskLastConeIsNotOfType, lastConeInEachConfigIdxMasked] = -1

    # keep only configs with at least 3 cones
    maskLengthIsAtleast3 = (endConfigurations != -1).sum(axis=1) >= 3
    endConfigurations = endConfigurations[maskLengthIsAtleast3]
    # print(endConfigurations[0])
    # print(len(endConfigurations))
    # remove identical configurations
    endConfigurations = np.unique(endConfigurations, axis=0)
    # print(endConfigurations[0])
    # print(len(endConfigurations))
    # remove subsets
    areEqualMask = endConfigurations[:, None] == endConfigurations
    areMinus1Mask = endConfigurations == -1
    areEqualMask = areEqualMask | areMinus1Mask

    isDuplicate = areEqualMask.all(axis=-1).sum(axis=0) > 1

    endConfigurations = endConfigurations[~isDuplicate]

    if len(endConfigurations) == 0:
        raise NoPathError("Could not create a valid trace using the provided points")

    return endConfigurations, allConfigurationsAndItsEndConfigurationIndicator


def adjacecnyMatrixToBordersAndTargets(
    adjacencyMatrix: IntArray,
) -> Tuple[IntArray, IntArray]:
    """
    Convert an adjacency matrix to two flat arrays, one representing the neighbors of
    each node and one which indicates the starting index of each node in the neighbors
    array
    [
        [0 1 0]
        [1 1 1]
        [1 0 0]
    ]
    is converted to

    neighbors -> [1, 0, 1, 2, 0]
    - Edge from node 0 to node 1.
    - Edge from node 1 to node 0, 1, and 2.
    - Edge from node 2 to node 0.

    borders -> [0, 1, 4, 5]
    - The neighbors of node 0 in neighbors_flat are located at indices 0 to 1 (exclusive).
    - The neighbors of node 1 in neighbors_flat are located at indices 1 to 4 (exclusive).
    - The neighbors of node 2 in neighbors_flat are located at indices 4 to 5 (exclusive).

    Args:
        adjacency_matrix: The adjacency matrix to convert
    Returns:
        The neighbors and the starting position of each node in the neighbors array
    """
    source, neighborsFlat = np.where(adjacencyMatrix)

    borders: IntArray = np.zeros((adjacencyMatrix.shape[0], 2), dtype=np.int32) - 1
    borders[0] = [0, 0]

    for index in range(len(neighborsFlat)):
        sourceIdx = source[index]

        if borders[sourceIdx][0] == -1:
            borders[sourceIdx][0] = index

        borders[sourceIdx][1] = index + 1

    # for all the nodes that have no neighbors set their start and end to the previous node's end
    for i, value in enumerate(borders):
        if value[0] == -1:
            previous = borders[i - 1][1]
            borders[i] = [previous, previous]

    finalBorders = np.concatenate((borders[:, 0], borders[-1:, -1]))

    return neighborsFlat, finalBorders


def implFindAllEndConfigurations(
    trace: FloatArray,
    coneType: ConeTypes,
    startIdx: int,
    adjacecnyNeighbors: IntArray,
    adjacencyBorders: IntArray,
    targetLength: int,
    thresholdDirectionalAngle: float,
    thresholdAbsoluteAngle: float,
    firstKIndicesMustBe: IntArray,
    carPosition: FloatArray,
    carDirection: np.float_,
    carSize: float,
    storeAllEndConfigurations: bool,
) -> tuple[IntArray, Optional[tuple[IntArray, BoolArray]]]:
    """
    Finds all the possible paths up to length target length.
    Args:
        start_idx: The index of the starting node
        adjacency_neighbors: The indices of the sink for each edge
        adjacency_borders: The start position of the indices of each node
        target_length: The length of the path that the search is searching for
    Returns:
        A 2d array of configurations (indices) that define all the valid paths
    """
    endConfigurations: IntArray = np.full((10, targetLength), -1, dtype=np.int32)
    endConfigurationsPointer = 0
    stack: IntArray = np.zeros((10, 2), dtype=np.int32)
    stackEndPointer = 0
    currentAttempt: IntArray = np.zeros(targetLength, dtype=np.int32) - 1
    if len(firstKIndicesMustBe) > 0:
        pos = len(firstKIndicesMustBe) - 1
        currentAttempt[:pos] = firstKIndicesMustBe[:-1]
        stack[0] = [firstKIndicesMustBe[-1], pos]
    else:
        stack[0] = [startIdx, 0]

    if storeAllEndConfigurations:
        allConfigurationsCounter = 0
        allConfigurations = endConfigurations.copy()
        configurationsIsEnd = np.zeros(endConfigurations.shape[0], dtype=np.bool_)

    while stackEndPointer >= 0:
        # pop the index and the position from the stack
        nextIdx, positionInStack = stack[stackEndPointer]
        stackEndPointer -= 1

        # add the node to the current path
        currentAttempt[positionInStack] = nextIdx
        currentAttempt[positionInStack + 1 :] = -1

        # get the neighbors of the last node in the attempt
        neighbors = adjacecnyNeighbors[adjacencyBorders[nextIdx] : adjacencyBorders[nextIdx + 1]]
        # print(len(adjacecnyNeighbors))
        # print(adjacencyBorders)
        # print(nextIdx)
        canBeAdded = neighborBoolMaskCanBeAddedToAttempt(
            trace,
            coneType,
            currentAttempt,
            positionInStack,
            neighbors,
            thresholdDirectionalAngle,
            thresholdAbsoluteAngle,
            carPosition,
            carDirection,
            carSize,
        )

        hasValidNeighbors = positionInStack < targetLength - 1 and np.any(canBeAdded)
        # check that we haven't hit target length and that we have neighbors to add
        if hasValidNeighbors:
            for i, canBeAdded in enumerate(canBeAdded):
                if not canBeAdded:
                    continue

                stackEndPointer += 1

                stack = resizeStackIfNeeded(stack, stackEndPointer)
                stack[stackEndPointer] = [neighbors[i], positionInStack + 1]

        # leaf
        else:
            endConfigurations = resizeStackIfNeeded(endConfigurations, endConfigurationsPointer)

            endConfigurations[endConfigurationsPointer:] = currentAttempt.copy()

            endConfigurationsPointer += 1

        if storeAllEndConfigurations:
            allConfigurations = resizeStackIfNeeded(allConfigurations, allConfigurationsCounter)
            configurationsIsEnd = resizeStackIfNeeded(configurationsIsEnd, allConfigurationsCounter)
            allConfigurations[allConfigurationsCounter] = currentAttempt
            configurationsIsEnd[allConfigurationsCounter] = not hasValidNeighbors
            allConfigurationsCounter += 1

    returnValueEndConfigurations: IntArray = endConfigurations[:endConfigurationsPointer]

    # maskEndConfigurationsWithMoreThanTwoNodes = (returnValueEndConfigurations != -1).sum(axis=1) > 2
    # returnValueEndConfigurations = returnValueEndConfigurations[
    #     maskEndConfigurationsWithMoreThanTwoNodes
    # ]

    if storeAllEndConfigurations:
        allConfigurations = allConfigurations[:allConfigurationsCounter]
        configurationsIsEnd = configurationsIsEnd[:allConfigurationsCounter]

    configHistory = None
    if storeAllEndConfigurations:
        configHistory = (allConfigurations, configurationsIsEnd)

    return returnValueEndConfigurations, configHistory


# for numba
FLOAT = float if TYPE_CHECKING else np.float32


def neighborBoolMaskCanBeAddedToAttempt(
    trace: FloatArray,
    coneType: ConeTypes,
    currentAttempt: IntArray,
    positionInStack: int,
    neighbors: IntArray,
    thresholdDirectionalAngle: float,
    thresholdAbsoluteAngle: float,
    carPosition: FloatArray,
    carDirection: np.float_,
    carSize: float,
) -> BoolArray:
    """
    Determines whether a neighbor cone can be added to the current attempt.

    Args:
        trace (FloatArray): Array of cone positions.
        coneType (ConeTypes): Type of cone (LEFT or RIGHT).
        currentAttempt (IntArray): Array of cone indices in the current attempt.
        positionInStack (int): Index of the current cone in the attempt.
        neighbors (IntArray): Array of neighbor cone indices.
        thresholdDirectionalAngle (float): Maximum angle in a specific direction for blue cones (counter-clockwise) 
                                           or yellow cones (clockwise).
        thresholdAbsoluteAngle (float): Absolute angle between two vectors.
        carPosition (FloatArray): Position of the car.
        carDirection (np.float_): Direction of the car.
        carSize (float): Size of the car.

    Returns:
        BoolArray: Array indicating whether each neighbor cone can be added to the attempt.
    """
    carDirection2d = unit2dVectorFromAngle(np.array(carDirection))
    carDirectionNormalized = carDirection2d / np.linalg.norm(carDirection2d)

    # neighbor can be added if not in current attempt
    canBeAdded = ~myIn1d(neighbors, currentAttempt[: positionInStack + 1])
    neighborsPoints = trace[neighbors]
    if positionInStack >= 1:
        maskInEllipse = calculateMaskWithinEllipse(
            trace, currentAttempt, positionInStack, neighborsPoints
        )

        canBeAdded = canBeAdded & maskInEllipse

    if positionInStack == 0:
        # the second cone in the attempt should be on the expected side pf the car (left cone on the left side of the car and right cone on the right side of the car)
        maskSecondConeRightSide = maskSecondInAttemptIsOnRightVehicleSide(
            coneType, carPosition, carDirectionNormalized, neighborsPoints
        )

        canBeAdded = canBeAdded & maskSecondConeRightSide

    for i in range(len(canBeAdded)):
        if not canBeAdded[i]:
            continue

        candidateNeighbor = neighbors[i]

        # find if there is a cone that is between the last cone in the attempt and the
        # candidate neighbor, if so we do not want to persue this path, because it will
        # skip one cone
        checkIfNeighborLiesBetweenLastInAttemptAndCandidate(
            trace,
            currentAttempt,
            positionInStack,
            neighbors,
            canBeAdded,
            i,
            candidateNeighbor,
        )

        candidateNeighborPos = trace[neighbors[i]]
        # calculate angle between second to last to last vector in attempt and the vector
        # between the last node and the candidate neighbor
        # add to current attempt only if the angle between the current last vector and
        # the potential new last vector is less than a specific threshold. There are two
        # thresholds, one is the maximum angle in a specific direction for blue cones that
        # is counter-clockwise and for yellow cones that is clockwise the second threshold
        # is an absolute angle between the two vectors.

        if canBeAdded[i] and positionInStack >= 1:
            secondToLastInAttempt = trace[currentAttempt[positionInStack - 1]]
            lastInAttempt = trace[currentAttempt[positionInStack]]
            secondToLastToLast = lastInAttempt - secondToLastInAttempt
            lastToCandidate = candidateNeighborPos - lastInAttempt
            angle1 = cast(
                FLOAT,
                np.arctan2(secondToLastToLast[1], secondToLastToLast[0]),
            )
            angle2 = cast(
                FLOAT,
                np.arctan2(lastToCandidate[1], lastToCandidate[0]),
            )
            # order is important here
            diffrence = angleDiffrence(angle2, angle1)
            lenLastToCandidate = np.linalg.norm(lastToCandidate)

            if np.abs(diffrence) > thresholdAbsoluteAngle:
                canBeAdded[i] = False
            elif coneType == ConeTypes.LEFT:
                canBeAdded[i] = diffrence < thresholdDirectionalAngle or lenLastToCandidate < 4.0
            elif coneType == ConeTypes.RIGHT:
                canBeAdded[i] = diffrence > -thresholdDirectionalAngle or lenLastToCandidate < 4.0
            else:
                raise AssertionError("Unreachable cone")

            # check if cone candidate causes change in direction in attempt
            if positionInStack >= 2:
                thirdToLast = trace[currentAttempt[positionInStack - 2]]
                thirdToLastToSecondToLast = secondToLastInAttempt - thirdToLast
                angle3 = cast(
                    FLOAT,
                    np.arctan2(
                        thirdToLastToSecondToLast[1],
                        thirdToLastToSecondToLast[0],
                    ),
                )
                diffrence2 = angleDiffrence(angle1, angle3)

                if (
                    np.sign(diffrence2) != np.sign(diffrence2)
                    and np.abs(diffrence - diffrence2) > 1.3
                ):
                    canBeAdded[i] = False

            if canBeAdded[i] and positionInStack == 1:
                start = trace[currentAttempt[0]]
                diff = candidateNeighborPos - start
                directionOffset = vecAngleBetween(np.array(unit2dVectorFromAngle(carDirection)), np.array(diff))
                canBeAdded[i] &= directionOffset < np.pi / 2

            if canBeAdded[i] and positionInStack >= 0:
                # make sure that no intersection with car occurs
                lastInAttempt = trace[currentAttempt[positionInStack]]
                carStart = carPosition - carDirectionNormalized * carSize / 2  #
                carEnd = carPosition + carDirectionNormalized * carSize  #

                canBeAdded[i] &= not linesSegmentsIntersectIndicator(
                    lastInAttempt, candidateNeighborPos, carStart, carEnd
                )

    return canBeAdded


def calculateMaskWithinEllipse(
    trace: FloatArray,
    currentAttempt: IntArray,
    positionInStack: int,
    neighborsPoints: FloatArray,
) -> BoolArray:
    lastInAttempt = trace[currentAttempt[positionInStack]]
    secondToLastInAttempt = trace[currentAttempt[positionInStack - 1]]
    secondToLastToLast = lastInAttempt - secondToLastInAttempt

    maskInEllipse = pointsInsideEllipse(
        neighborsPoints,
        lastInAttempt,
        major_direction=secondToLastToLast,
        major_radius=6,
        minor_radius=3,
    )

    return maskInEllipse


def maskSecondInAttemptIsOnRightVehicleSide(
    coneType: ConeTypes,
    carPosition: FloatArray,
    carDirectionNormalized: FloatArray,
    neighborsPoints: FloatArray,
) -> BoolArray:
    carToNeighbors = neighborsPoints - carPosition
    angleCarDir = np.arctan2(carDirectionNormalized[1], carDirectionNormalized[0])
    angleCarToNeighbors = np.arctan2(carToNeighbors[:, 1], carToNeighbors[:, 0])

    angleDiff = angleDiffrence(angleCarToNeighbors, angleCarDir)

    expectedSign = 1 if coneType == ConeTypes.LEFT else -1

    maskExpectedSide = np.sign(angleDiff) == expectedSign
    maskOtherSideTolerance = np.abs(angleDiff) < np.deg2rad(12) #was5

    mask = maskExpectedSide | maskOtherSideTolerance
    return mask


def angleDiffrence(angle1: FloatArray, angle2: FloatArray) -> FloatArray:
    """
    Calculate the difference between two angles. The range of the difference is [-pi, pi].
    The order of the angles *is* important.

    Args:
        angle1: First angle.
        angle2: Second angle.

    Returns:
        The difference between the two angles.
    """
    return cast(FloatArray, (angle1 - angle2 + 3 * np.pi) % (2 * np.pi) - np.pi)


def checkIfNeighborLiesBetweenLastInAttemptAndCandidate(
    trace: FloatArray,
    currentAttempt: IntArray,
    positionInStack: int,
    neighbors: IntArray,
    canBeAdded: BoolArray,
    i: int,
    candidateNeighbor: int,
):
    for neighbor in neighbors:
        if neighbor == neighbors[i]:
            continue

        neighborToLastInAttempt = trace[currentAttempt[positionInStack]] - trace[neighbor]

        neighborToCandidate = trace[candidateNeighbor] - trace[neighbor]

        distToCandidate = np.linalg.norm(neighborToCandidate)
        distToLastInAttempt = np.linalg.norm(neighborToLastInAttempt)

        # if the angle between the 2 vectors is more than 150 degrees then the neighbor
        # cone is between the last cone in the attemot and the candidate neighbor to the
        # attempt
        if (
            distToCandidate < 6.0
            and distToLastInAttempt < 6.0
            and vecAngleBetween(neighborToLastInAttempt + 0.000001, neighborToCandidate + 0.000001) > np.deg2rad(150)
        ):
            canBeAdded[i] = False
            break


_DEFAULT_EPSILON = 1e-6


def linesSegmentsIntersectIndicator(
    segmentAStart: FloatArray,
    segmentAEnd: FloatArray,
    segmentBStart: FloatArray,
    segmentBEnd: FloatArray,
    epsilon: float = _DEFAULT_EPSILON,
) -> bool:
    """
    Given the start and endpoint of two 2d-line segments indicate if the two line segments intersect.

    Args:
        segment_a_start: The start point of the first line segment.
        segment_a_end: The end point of the first line segment.
        segment_b_start: The start point of the second line segment.
        segment_b_end: The end point of the second line segment.
        epsilon: The epsilon to use for the comparison.

    Returns:
        A boolean indicating if the two line segments intersect.
    """
    # Adapted from https://stackoverflow.com/a/42727584
    homogeneous = _makeSegmentsHomogeneous(segmentAStart, segmentAEnd, segmentBStart, segmentBEnd)

    lineA = np.cross(homogeneous[0], homogeneous[1])  # get first line
    lineB = np.cross(homogeneous[2], homogeneous[3])  # get second line
    interX, interY, interZ = np.cross(lineA, lineB)  # point of intersection

    # lines are parallel <=> z is zero
    if np.abs(interZ) < epsilon:
        return _handleLineSegmentsIntersectParallelCase(
            segmentAStart, segmentAEnd, segmentBStart, segmentBEnd
        )

    # find intersection point
    intersectionX, intersectionY = np.array([interX / interZ, interY / interZ])

    # bounding boxes
    segmentALeft, segmentARight = np.sort(homogeneous[:2, 0])
    segmentBLeft, segmentBRight = np.sort(homogeneous[2:, 0])
    segmentABottom, segmentATop = np.sort(homogeneous[:2, 1])
    segmentBBottom, segmentBTop = np.sort(homogeneous[2:, 1])

    # check that intersection point is in both bounding boxes
    # check with a bit of epsilon for numerical stability

    returnValue = (
        (segmentALeft - epsilon <= intersectionX <= segmentARight + epsilon)
        and (segmentBLeft - epsilon <= intersectionX <= segmentBRight + epsilon)
        and (segmentABottom - epsilon <= intersectionY <= segmentATop + epsilon)
        and (segmentBBottom - epsilon <= intersectionY <= segmentBTop + epsilon)
    )

    return bool(returnValue)


def _makeSegmentsHomogeneous(
    segmentAStart: FloatArray,
    segmentAEnd: FloatArray,
    segmentBStart: FloatArray,
    segmentBEnd: FloatArray,
) -> FloatArray:
    homogeneous = np.ones((4, 3))
    homogeneous[0, :2] = segmentAStart
    homogeneous[1, :2] = segmentAEnd
    homogeneous[2, :2] = segmentBStart
    homogeneous[3, :2] = segmentBEnd

    return homogeneous


def _handleLineSegmentsIntersectParallelCase(
    segmentAStart: FloatArray,
    segmentAEnd: FloatArray,
    segmentBStart: FloatArray,
    segmentBEnd: FloatArray,
    epsilon: float,
) -> bool:
    # lines are parallel only one slope calculation nessessary
    diffrence: FloatArray = segmentAEnd - segmentAStart

    if diffrence[0] < epsilon:
        # parallel vertical lines segments
        # overlap only possible if x element is the same for both line segments
        maybeOverlap = np.abs(segmentAStart[0] - segmentBStart[0]) < epsilon
        slope = np.inf
    else:
        slope = diffrence[1] / diffrence[0]
        # parallel non verticle lines
        # overlap only possible if x element is the same for both line segments
        interceptA = segmentAStart[1] - slope * segmentAStart[0]
        interceptB = segmentBStart[1] - slope * segmentBStart[0]
        maybeOverlap = np.abs(interceptA - interceptB) < epsilon

    if not maybeOverlap:
        # parallel lines, diffrent intercept, 100% no intercection
        return False

    axisToUse = 1 if slope > 1 else 0

    if segmentAStart[axisToUse] < segmentBStart[axisToUse]:
        leftSegmentEndScalar = segmentAEnd[axisToUse]
        rightSegmentStartScalar = min(segmentBStart[axisToUse], segmentBEnd[axisToUse])
    else:
        leftSegmentEndScalar = segmentBEnd[axisToUse]
        rightSegmentStartScalar = min(segmentAStart[axisToUse], segmentAEnd[axisToUse])

    returnValue = cast(
        bool,
        leftSegmentEndScalar >= rightSegmentStartScalar,
    )
    return returnValue


def resizeStackIfNeeded(stack: GenericArray, stackPointer: int) -> GenericArray:
    """
    Given a stack and its current pointer resize the stack if adding one more element
    would result in an index error
    Args:
        stack: the stack to potentially resize
        stackPointer: The current position (filled size) of the stack
    Returns:
        np.ndarray: The stack, if resized then a copy is returned
    """
    if stackPointer >= stack.shape[0]:
        stack = doubleStackLen(stack)

    return stack


def doubleStackLen(stack: GenericArray) -> GenericArray:
    """
    Double the capacity of a stack (used in path search)
    Args:
        stack: The stack to double
    Returns:
        A copy of the stack with double the capacity
    """
    _len = stack.shape[0]
    newShape = (_len * 2, *stack.shape[1:])
    newBuffer = np.full(newShape, -1, dtype=stack.dtype)

    newBuffer[:_len] = stack
    return newBuffer


class NoPathError(RuntimeError):
    """
    A special exception thrown when no path can be found (i.e no configuration)
    """
