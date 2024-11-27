"""
Description: This File calculates all the possible paths
"""

from typing import TYPE_CHECKING, Tuple, cast, Any, Union

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
    configurationParameters: Tuple[int, int, float, float],
    adjacencyMatrix: IntArray,
    firstKIndicesMustBe: IntArray,
    vehicleOdometry: tuple[FloatArray, np.float_],
) -> IntArray:
    """
    Finds all the possible paths that include all the reachable nodes from the starting
    Args:
        adjacency_matrix: The adjacency matrix indicating which nodes are
                          connected to which
        configurationParameters[0]: startIdx: The index of the starting node
        configurationParameters[1]: targetLength: The length of the path
                                    that the search is searching for
        configurationParameters[2]: thresholdDirectionalAngle
        configurationParameters[3]: thresholdAbsoluteAngle
    Raises:
        NoPathError: If no path has been found
    Returns:
        A 2d array of configurations (indices) that define a path
    """

    pointsXY = points[:, :2]

    endConfigurations: np.ndarray[Any, np.dtype[np.int_]] = implFindAllEndConfigurations(
        pointsXY,
        coneType,
        configurationParameters,
        adjacencyMatrix,
        firstKIndicesMustBe,
        vehicleOdometry,
    )

    if len(firstKIndicesMustBe) > 0 and len(endConfigurations) > 0:
        endConfigurations = endConfigurations[
            np.array(
                (endConfigurations[:, : len(firstKIndicesMustBe)] == firstKIndicesMustBe).all(
                    axis=1
                )
            )
        ]

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

    # remove identical configurations
    endConfigurations = np.unique(endConfigurations, axis=0)

    # remove subsets
    areEqualMask = endConfigurations[:, None] == endConfigurations
    areEqualMask = areEqualMask | (endConfigurations == -1)

    isDuplicate = areEqualMask.all(axis=-1).sum(axis=0) > 1

    endConfigurations = endConfigurations[~isDuplicate]

    if len(endConfigurations) == 0:
        raise NoPathError("Could not create a valid trace using the provided points")

    return endConfigurations


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
    configurationParameters: Tuple[int, int, float, float],
    adjacencyMatrix: IntArray,
    firstKIndicesMustBe: IntArray,
    carOdometry: Tuple[FloatArray, np.float_],
) -> IntArray:
    """
    Finds all the possible paths up to length target length.
    Args:
        configurationParameters[0] = start_idx: The index of the starting node
        adjacency_neighbors: The indices of the sink for each edge
        adjacency_borders: The start position of the indices of each node
        target_length: The length of the path that the search is searching for
    Returns:
        A 2d array of configurations (indices) that define all the valid paths
    """
    targetLength = int(configurationParameters[1])
    stack: IntArray = np.zeros((10, 2), dtype=np.int32)
    currentAttempt: IntArray = np.zeros(targetLength, dtype=np.int32) - 1
    if len(firstKIndicesMustBe) > 0:
        pos = len(firstKIndicesMustBe) - 1
        currentAttempt[:pos] = firstKIndicesMustBe[:-1]
        stack[0] = [firstKIndicesMustBe[-1], pos]
    else:
        # We will use the startIdx
        stack[0] = [int(configurationParameters[0]), 0]

    return findAllEndConfigurationsLoop(
        trace,
        coneType,
        configurationParameters,
        adjacencyMatrix,
        stack,
        currentAttempt,
        carOdometry,
    )


def findAllEndConfigurationsLoop(
    trace: FloatArray,
    coneType: ConeTypes,
    configurationParameters: Tuple[int, int, float, float],
    adjacencyMatrix: IntArray,
    stack: IntArray,
    currentAttempt: IntArray,
    carOdometry: Tuple[FloatArray, np.float_],
) -> np.ndarray[np.int32, Any]:
    """
    Finds all possible end configurations using a loop-based approach.

    Args:
        trace (FloatArray): The trace data.
        coneType (ConeTypes): The type of cone.
        configurationParameters (FloatArray): The configuration parameters.
        adjacencyMatrix (IntArray): The adjacency matrix.
        stack (IntArray): The stack data structure.
        currentAttempt (IntArray): The current attempt.
        carOdometry: The car odometry data.

    Returns:
        tuple[IntArray]: A tuple containing the end configurations.
    """
    adjacecnyNeighbors, adjacencyBorders = adjacecnyMatrixToBordersAndTargets(adjacencyMatrix)
    endConfigurations: IntArray = np.full((10, int(configurationParameters[1])), -1, dtype=np.int32)
    pointers = [0, 0]  # endConfigurationsPointer, stackEndPointer
    while pointers[1] >= 0:
        # pop the index and the position from the stack
        nextIdx, positionInStack = stack[pointers[1]]
        pointers[1] -= 1

        # add the node to the current path
        currentAttempt[positionInStack] = nextIdx
        currentAttempt[positionInStack + 1 :] = -1

        # get the neighbors of the last node in the attempt
        canBeAdded = neighborBoolMaskCanBeAddedToAttempt(
            trace,
            coneType,
            currentAttempt,
            positionInStack,
            adjacecnyNeighbors[
                adjacencyBorders[nextIdx] : adjacencyBorders[nextIdx + 1]  # neighbors
            ],
            configurationParameters,
            carOdometry,
        )

        # check that we haven't hit target length and that we have neighbors to add
        if positionInStack < int(configurationParameters[1]) - 1 and np.any(canBeAdded):
            for i, canBeAdded in enumerate(canBeAdded):
                if not canBeAdded:
                    continue

                pointers[1] += 1

                stack = resizeStackIfNeeded(stack, pointers[1])
                stack[pointers[1]] = [
                    adjacecnyNeighbors[adjacencyBorders[nextIdx] : adjacencyBorders[nextIdx + 1]][
                        i
                    ],
                    positionInStack + 1,
                ]

        # leaf
        else:
            endConfigurations = resizeStackIfNeeded(endConfigurations, pointers[0])

            endConfigurations[pointers[0] :] = currentAttempt.copy()

            pointers[0] += 1

    return np.array(endConfigurations[: pointers[0]]).astype(np.int32)


# for numba
FLOAT = Union[float, np.float32] if TYPE_CHECKING else np.float32


def neighborBoolMaskCanBeAddedToAttempt(
    trace: FloatArray,
    coneType: ConeTypes,
    currentAttempt: IntArray,
    positionInStack: int,
    neighbors: IntArray,
    configurationParameters: Tuple[int, int, float, float],
    carOdometry: Tuple[FloatArray, np.float_],
) -> BoolArray:
    """
    Determines whether a neighbor cone can be added to the current attempt.

    Args:
        trace (FloatArray): Array of cone positions.
        coneType (ConeTypes): Type of cone (LEFT or RIGHT).
        currentAttempt (IntArray): Array of cone indices in the current attempt.
        positionInStack (int): Index of the current cone in the attempt.
        neighbors (IntArray): Array of neighbor cone indices.
        configurationParameters[2] = thresholdDirectionalAngle (float): Maximum angle
                                           in a specific direction for blue cones
                                           (counter-clockwise) or yellow cones (clockwise).
        configurationParameters[3] = thresholdAbsoluteAngle (float): Absolute angle between
                                     two vectors.
        carOdometry[0] = carPosition (FloatArray): Position of the car.
        carOdometry[1] = carDirection (np.float_): Direction of the car.

    Returns:
        BoolArray: Array indicating whether each neighbor cone can be added to the attempt.
    """
    carDirectionNormalized = unit2dVectorFromAngle(np.array(carOdometry[1])) / np.linalg.norm(
        unit2dVectorFromAngle(np.array(carOdometry[1]))
    )

    # neighbor can be added if not in current attempt
    canBeAdded = ~myIn1d(neighbors, currentAttempt[: positionInStack + 1])
    if positionInStack >= 1:

        canBeAdded = canBeAdded & (
            calculateMaskWithinEllipse(trace, currentAttempt, positionInStack, trace[neighbors])
        )

    if positionInStack == 0:
        # the second cone in the attempt should be on the expected side of the car
        #  (left cone on the left side of the car and right cone on the right side of the car)

        canBeAdded = canBeAdded & (
            maskSecondInAttemptIsOnRightVehicleSide(
                coneType, carOdometry[0], carDirectionNormalized, trace[neighbors]
            )
        )

    for i, _ in enumerate(canBeAdded):
        if not canBeAdded[i]:
            continue

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
            neighbors[i],  # candidate neighbor
        )

        # calculate angle between second to last to last vector in attempt and the vector
        # between the last node and the candidate neighbor
        # add to current attempt only if the angle between the current last vector and
        # the potential new last vector is less than a specific threshold. There are two
        # thresholds, one is the maximum angle in a specific direction for blue cones that
        # is counter-clockwise and for yellow cones that is clockwise the second threshold
        # is an absolute angle between the two vectors.

        if canBeAdded[i] and positionInStack >= 1:
            secondToLastToLast = (
                trace[currentAttempt[positionInStack]] - trace[currentAttempt[positionInStack - 1]]
            )
            lastToCandidate = trace[neighbors[i]] - trace[currentAttempt[positionInStack]]
            # order is important here
            # angle2 - angle1
            diffrence = angleDiffrence(
                np.arctan2(lastToCandidate[1], lastToCandidate[0]).astype(FLOAT),
                np.arctan2(secondToLastToLast[1], secondToLastToLast[0]).astype(FLOAT),
            )

            if np.abs(diffrence) > configurationParameters[3]:
                canBeAdded[i] = False
            elif coneType == ConeTypes.left:
                canBeAdded[i] = (
                    diffrence < configurationParameters[2] or np.linalg.norm(lastToCandidate) < 4.0
                )
            elif coneType == ConeTypes.right:
                canBeAdded[i] = (
                    diffrence > -configurationParameters[2] or np.linalg.norm(lastToCandidate) < 4.0
                )

            # check if cone candidate causes change in direction in attempt
            if positionInStack >= 2:
                diffrence2 = angleDiffrence(
                    np.arctan2(secondToLastToLast[1], secondToLastToLast[0]).astype(FLOAT),
                    np.arctan2(  # thirdToLastToSecondToLast
                        (
                            trace[currentAttempt[positionInStack - 1]]
                            - trace[currentAttempt[positionInStack - 2]]
                        )[1],
                        (
                            trace[currentAttempt[positionInStack - 1]]
                            - trace[currentAttempt[positionInStack - 2]]
                        )[0],
                    ).astype(FLOAT),
                )

                if (
                    np.sign(diffrence2) != np.sign(diffrence2)
                    and np.abs(diffrence - diffrence2) > 1.3
                ):
                    canBeAdded[i] = False

            if canBeAdded[i] and positionInStack == 1:
                # and with the canBeAdded the direction offset that is less than pi/2
                canBeAdded[i] &= (
                    vecAngleBetween(
                        np.array(unit2dVectorFromAngle(np.array(carOdometry[1]))),
                        np.array(trace[neighbors[i]] - trace[currentAttempt[0]]),
                    )
                    < np.pi / 2
                )

            if canBeAdded[i] and positionInStack >= 0:
                # make sure that no intersection with car occurs
                # 2.1 is the carSize
                canBeAdded[i] &= not linesSegmentsIntersectIndicator(
                    trace[currentAttempt[positionInStack]],  # last in attempt
                    trace[neighbors[i]],
                    carOdometry[0] - carDirectionNormalized * 2.1 / 2,  # car start
                    carOdometry[0] + carDirectionNormalized * 2.1,  # car end
                )

    return canBeAdded


def calculateMaskWithinEllipse(
    trace: FloatArray,
    currentAttempt: IntArray,
    positionInStack: int,
    neighborsPoints: FloatArray,
) -> BoolArray:
    """
    Calculates a mask indicating which points in the neighborsPoints array are within an ellipse.

    Args:
        trace (FloatArray): Array of trace points.
        currentAttempt (IntArray): Array of indices representing the current attempt.
        positionInStack (int): The position of the current attempt in the stack.
        neighborsPoints (FloatArray): Array of neighboring points.

    Returns:
        BoolArray: Array indicating which points are within the ellipse.
    """
    lastInAttempt = trace[currentAttempt[positionInStack]]
    secondToLastInAttempt = trace[currentAttempt[positionInStack - 1]]
    secondToLastToLast = lastInAttempt - secondToLastInAttempt

    maskInEllipse = pointsInsideEllipse(
        neighborsPoints,
        lastInAttempt,
        majorDirection=secondToLastToLast,
        majorRadius=6,
        minorRadius=3,
    )

    return maskInEllipse


def maskSecondInAttemptIsOnRightVehicleSide(
    coneType: ConeTypes,
    carPosition: FloatArray,
    carDirectionNormalized: FloatArray,
    neighborsPoints: FloatArray,
) -> BoolArray:
    """
    Determines the mask indicating whether the second cone in the attempt is on the right side
    of the vehicle.

    Args:
        coneType (ConeTypes): The type of cone (left or right).
        carPosition (FloatArray): The position of the car.
        carDirectionNormalized (FloatArray): The normalized direction vector of the car.
        neighborsPoints (FloatArray): The positions of the neighboring cones.

    Returns:
        BoolArray: A boolean array indicating whether the second cone is on the right side
        of the vehicle.
    """
    carToNeighbors = neighborsPoints - carPosition
    angleCarDir = np.arctan2(carDirectionNormalized[1], carDirectionNormalized[0])
    angleCarToNeighbors = np.arctan2(carToNeighbors[:, 1], carToNeighbors[:, 0])

    angleDiff = angleDiffrence(angleCarToNeighbors, angleCarDir)

    expectedSign = 1 if coneType == ConeTypes.left else -1

    maskExpectedSide = np.sign(angleDiff) == expectedSign
    maskOtherSideTolerance = np.abs(angleDiff) < np.deg2rad(12)  # was5

    mask = maskExpectedSide | maskOtherSideTolerance
    return np.asarray(mask, dtype=bool)


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
) -> None:
    """
    Checks if a neighbor lies between the last cone in the attempt and the candidate neighbor.

    Args:
        trace (FloatArray): Array of trace values.
        currentAttempt (IntArray): Array of indices representing the current attempt.
        positionInStack (int): Position of the current attempt in the stack.
        neighbors (IntArray): Array of neighbor indices.
        canBeAdded (BoolArray): Array indicating whether a neighbor can be added.
        i (int): Index of the current neighbor.
        candidateNeighbor (int): Index of the candidate neighbor.

    Returns:
        None
    """
    for neighbor in neighbors:
        if neighbor == neighbors[i]:
            continue

        neighborToLastInAttempt = trace[currentAttempt[positionInStack]] - trace[neighbor]

        neighborToCandidate = trace[candidateNeighbor] - trace[neighbor]

        distToCandidate = np.linalg.norm(neighborToCandidate)
        distToLastInAttempt = np.linalg.norm(neighborToLastInAttempt)

        # if the angle between the 2 vectors is more than 150 degrees then the neighbor
        # cone is between the last cone in the attempt and the candidate neighbor to the
        # attempt
        if (
            distToCandidate < 6.0
            and distToLastInAttempt < 6.0
            and vecAngleBetween(neighborToLastInAttempt + 0.000001, neighborToCandidate + 0.000001)
            > np.deg2rad(150)
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
    Given the start and endpoint of two 2d-line segments indicate
    if the two line segments intersect.

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

    # point of intersection
    crossProduct: np.ndarray[float, Any] = np.cross(
        np.cross(homogeneous[0], homogeneous[1]),  # get first line
        np.cross(homogeneous[2], homogeneous[3]),  # get second line
    )
    interX, interY, interZ = crossProduct.flatten()
    # lines are parallel <=> z is zero
    if np.abs(interZ) < epsilon:
        return _handleLineSegmentsIntersectParallelCase(
            segmentAStart, segmentAEnd, segmentBStart, segmentBEnd
        )

    # find intersection point
    intersectionX, intersectionY = np.array([interX / interZ, interY / interZ])

    return calcBoundingBox(homogeneous, intersectionX, intersectionY)


def calcBoundingBox(
    homogeneous: FloatArray,
    intersectionX: float,
    intersectionY: float,
    epsilon: float = _DEFAULT_EPSILON,
) -> bool:
    """
    Calculates the bounding box for a given set of homogeneous coordinates and
    checks if the intersection point lies within the bounding boxes.

    Args:
        homogeneous (FloatArray): The homogeneous coordinates of the segments.
        intersectionX (float): The x-coordinate of the intersection point.
        intersectionY (float): The y-coordinate of the intersection point.
        epsilon (float, optional): A small value used for numerical stability.
        Defaults to _DEFAULT_EPSILON.

    Returns:
        bool: True if the intersection point lies within both bounding boxes,
        False otherwise.
    """
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
    epsilon: float = _DEFAULT_EPSILON,
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
    newBuffer: np.ndarray[np.int_, Any] = np.full(newShape, -1, dtype=stack.dtype)

    newBuffer[:_len] = stack
    return newBuffer


class NoPathError(RuntimeError):
    """
    A special exception thrown when no path can be found (i.e no configuration)
    """
