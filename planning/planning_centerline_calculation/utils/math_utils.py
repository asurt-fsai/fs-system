#!/usin_roll/bin/env python3
# -*- coding:utf-8 -*-
"""
Description: A module with common mathematical functions
"""
from typing import TypeVar, cast

import math
import numpy as np
from numpy.typing import NDArray
from numba import jit

T = TypeVar("T")


def myNjit(func: T) -> T:
    """
    numba.njit is an untyped decorator. This wrapper helps type checkers keep the
    type information after applying the decorator. Furthermore, it sets some performance
    flags

    Args:
        func (T): The function to jit

    Returns:
        T: The jitted function
    """
    jitFunc: T = jit(nopython=True, cache=True, nogil=True, fastmath=True)(func)

    return jitFunc


@myNjit
def vecDot(vecs1: np.ndarray, vecs2: np.ndarray) -> np.ndarray:
    """
    Mutliplies vectors in an array elementwise

    Args:
        vecs1 (np.array): The first "list" of vectors
        vecs2 (np.array): The second "list" of vectors

    Returns:
        np.array: The results
    """
    return np.sum(vecs1 * vecs2, axis=-1)


@myNjit
def normOfLastAxis(arr: np.ndarray) -> np.ndarray:
    """
    Calculates the Euclidean norm of each vector along the last axis 
    of a NumPy array and returns a new array with the same shape as the input 
    array excluding the last dimension.

    Args:
        arr (np.ndarray): The input NumPy array.

    Returns:
        np.ndarray: A new array containing the L2 norm of each vector 
                    along the last axis of the original array.
    """
    originalShape = arr.shape
    arrRowCol = np.ascontiguousarray(arr).reshape(-1, arr.shape[-1])
    result = np.empty(arrRowCol.shape[0])
    for i in range(arrRowCol.shape[0]):
        vec = arrRowCol[i]
        result[i] = np.sqrt(vecDot(vec, vec))

    result = result.reshape(originalShape[:-1])

    return result

def angleToVector(angle: np.float_) -> NDArray[np.float_]:
    """
    Converts an angle in radians to a 2D unit vector.

    Args:
        angle: The angle in radians.

    Returns:
        A 2D unit vector representing the direction of the angle.
    """

    x = math.cos(angle)
    y = math.sin(angle)

    # Normalize to create a unit vector
    magnitude = math.sqrt(x**2 + y**2)
    x /= magnitude
    y /= magnitude
    unitVector = np.array([x / magnitude, y / magnitude], dtype=np.float_)

    return unitVector

@myNjit
def vecAngleBetween(
    vecs1: np.ndarray, vecs2: np.ndarray, clipCosTheta: bool = True
) -> np.ndarray:
    """
    Calculates the angle between the vectors of the last dimension

    Args:
        vecs1 (np.ndarray): An array of shape (...,2)
        vecs2 (np.ndarray): An array of shape (...,2)
        clip_cos_theta (bool): Clip the values of the dot products so that they are
        between -1 and 1. Defaults to True.

    Returns:
        np.ndarray: A vector, such that each element i contains the angle between
        vectors vecs1[i] and vecs2[i]
    """

    cosTheta = vecDot(vecs1, vecs2)

    cosTheta /= normOfLastAxis(vecs1) * normOfLastAxis(vecs2)

    cosTheta = np.asarray(cosTheta)

    cosThetaFlat = cosTheta.ravel()

    if clipCosTheta:
        cosThetaFlat[cosThetaFlat < -1] = -1
        cosThetaFlat[cosThetaFlat > 1] = 1

    return np.arccos(cosTheta)


@myNjit
def rotate(points: np.ndarray, theta: float) -> np.ndarray:
    """
    Rotates the points in `points` by angle `theta` around the origin

    Args:
        points (np.array): The points to rotate. Shape (n,2)
        theta (float): The angle by which to rotate in radians

    Returns:
        np.array: The points rotated
    """
    cosTheta, sinTheta = np.cos(theta), np.sin(theta)
    rotationMatrix = np.array(((cosTheta, -sinTheta), (sinTheta, cosTheta))).T
    return np.dot(points, rotationMatrix)


@myNjit
def myCdistSqEuclidean(arrA: np.ndarray, arrB: np.ndarray) -> np.ndarray:
    """
    Calculates the pairwise square euclidean distances from each point in `X` to each
    point in `Y`

    Credit:
        Uses https://stackoverflow.com/a/56084419 which in turn uses
        https://github.com/droyed/eucl_dist

    Args:
        arr_a (np.array): A 2d array of shape (m,k)
        arr_b (np.array): A 2d array of shape (n,k)

    Returns:
        np.array: A matrix of shape (m,n) containing the square euclidean distance
        between all the points in `X` and `Y`
    """
    nX, dim = arrA.shape
    xExt = np.empty((nX, 3 * dim))
    xExt[:, :dim] = 1
    xExt[:, dim : 2 * dim] = arrA
    xExt[:, 2 * dim :] = np.square(arrA)

    nY = arrB.shape[0]
    yExt = np.empty((3 * dim, nY))
    yExt[:dim] = np.square(arrB).T
    yExt[dim : 2 * dim] = -2 * arrB.T
    yExt[2 * dim :] = 1

    return np.dot(xExt, yExt)


@myNjit
def calcPairwiseDistances(
    points: np.ndarray, distToSelf: float = 0.0
) -> np.ndarray:
    """
    Given a set of points, creates a distance matrix from each point to every point

    Args:
        points (np.ndarray): The points for which the distance matrix should be
        calculated dist_to_self (np.ndarray, optional): The distance to set the
        diagonal. Defaults to 0.0.

    Returns:
        np.ndarray: The 2d distance matrix
    """
    pairwiseDistances = myCdistSqEuclidean(points, points)

    if distToSelf != 0:
        for i in range(len(points)):
            pairwiseDistances[i, i] = distToSelf
    return pairwiseDistances


@myNjit
def myIn1d(testValues: np.ndarray, sourceContainer: np.ndarray) -> np.ndarray:
    """
    Calculate a boolean mask for a 1d array indicating if an element in `test_values` is
    present in `source container` which is also 1d

    Args:
        test_values (np.ndarray): The values to test if they are inside the container
        source_container (np.ndarray): The container

    Returns:
        np.ndarray: A boolean array with the same length as `test_values`. If
        `return_value[i]` is `True` then `test_value[i]` is in `source_container`
    """
    sourceSorted = np.sort(sourceContainer)
    isIn = np.zeros(testValues.shape[0], dtype=np.bool_)
    for i, testVal in enumerate(testValues):
        for sourceVal in sourceSorted:
            if testVal == sourceVal:
                isIn[i] = True
                break

            if sourceVal > testVal:
                break

    return isIn


def traceCalculateConsecutiveRadii(trace: np.ndarray) -> np.ndarray:
    """
    Expects a (n,2) array and returns the radius of the circle that passes
    between all consecutive point triples. The radius between index 0,1,2, then 1,2,3
    and so on

    Args:
        trace (np.ndarray): The points for which the radii will be calculated

    Returns:
        np.ndarray: The radii for each consecutive point triple
    """

    # TODO: Vectorize this function. Limit is the indexer
    indexer = np.arange(3)[None, :] + 1 * np.arange(trace.shape[-2] - 2)[:, None]

    points = trace[indexer]
    radii = calculateRadiusFromPoints(points)
    return radii


def traceDistanceToNext(trace: np.ndarray) -> np.ndarray:
    """
    Calculates the distance of one point in the trace to the next. Obviously the last
    point doesn't have any distance associated

    Args:
        trace (np.array): The points of the trace

    Returns:
        np.array: A vector containing the distances from one point to the next
    """
    return np.linalg.norm(np.diff(trace, axis=-2), axis=-1)


def traceAnglesBetween(trace: np.ndarray) -> np.ndarray:
    """
    Calculates the angles in a trace from each point to its next

    Args:
        trace (np.array): The trace containing a series of 2d vectors

    Returns:
        np.array: The angle from each vector to its next, with `len(return_value) ==
        len(trace) - 1`
    """
    allToNext = np.diff(trace, axis=-2)
    fromMiddleToNext = allToNext[..., 1:, :]
    fromMiddleToPrev = -allToNext[..., :-1, :]
    angles = vecAngleBetween(fromMiddleToNext, fromMiddleToPrev)
    return angles


@myNjit
def unit2dVectorFromAngle(rad: np.ndarray) -> np.ndarray:
    """
    Creates unit vectors for each value in the rad array

    Args:
        rad (np.array): The angles (in radians) for which the vectors should be created

    Returns:
        np.array: The created unit vectors
    """
    rad = np.asarray(rad)
    newShape = rad.shape + (2,)
    res = np.empty(newShape, dtype=rad.dtype)
    res[..., 0] = np.cos(rad)
    res[..., 1] = np.sin(rad)
    return res


# Calculates the angle of each vector in `vecs`
# TODO: Look into fixing return type when a single vector is provided (return float)
@myNjit
def angleFrom2dVector(vecs: np.ndarray) -> np.ndarray:
    """
    Calculates the angle of each vector in `vecs`. If `vecs` is just a single 2d vector
    then one angle is calculated and a scalar is returned

    >>> import numpy as np
    >>> x = np.array([[1, 0], [1, 1], [0, 1]])
    >>> angleFrom2dVector(x)
    >>> array([0.        , 0.78539816, 1.57079633])

    Args:
        vecs (np.array): The vectors for which the angle is calculated

    Raises:
        ValueError: If `vecs` has the wrong shape a ValueError is raised

    Returns:
        np.array: The angle of each vector in `vecs`
    """
    assert vecs.shape[-1] == 2, "vecs must be a 2d vector"

    vecsFlat = vecs.reshape(-1, 2)

    angles = np.arctan2(vecsFlat[:, 1], vecsFlat[:, 0])
    returnValue = angles.reshape(vecs.shape[:-1])

    # if vecs.ndim == 1:
    #     return return_value[0]

    return returnValue


@myNjit
def normalizeLastAxis(vecs: np.ndarray) -> np.ndarray:
    """
    Returns a normalized version of vecs

    Args:
        vecs (np.ndarray): The vectors to normalize
    Returns:
        np.ndarray: The normalized vectors
    """
    vecsFlat = vecs.reshape(-1, vecs.shape[-1])
    out = np.zeros(vecs.shape, dtype=vecs.dtype)
    for i, vec in enumerate(vecsFlat):
        out[i] = vec / np.linalg.norm(vec)

    return out.reshape(vecs.shape)


@myNjit
def lerp(
    valuesToLerp: np.ndarray,
    start1: np.ndarray,
    stop1: np.ndarray,
    start2: np.ndarray,
    stop2: np.ndarray,
) -> np.ndarray:
    """
    Linearly interpolates (lerps) from one sin_pitchace `[start1, stop1]` to another
    `[start2, stop2]`. `start1 >= stop1` and `start2 >= stop2` are allowed. If ns is a
    2d array, then start1, stop1, start2, stop2 must be 1d vectors. This allows for
    lerping in any n-dim sin_pitchace

    >>> import numpy as np
    >>> x = np.array([1, 2, 3])
    >>> lerp(x, 0, 10, 30, 100)
    >>> array([37., 44., 51.])

    Args:
        values_to_lerp (np.array): The points to interpolate
        start1 (np.array): The beginning of the original sin_pitchace
        stop1 (np.array): The end of the original sin_pitchace
        start2 (np.array): The beginning of the target sin_pitchace
        stop2 (np.array): The end of the target sin_pitchace

    Returns:
        np.array: The interpolated points
    """
    return (valuesToLerp - start1) / (stop1 - start1) * (stop2 - start2) + start2


def calculateRadiusFromPoints(points: np.ndarray) -> np.ndarray:
    """
    Given a three points this function calculates the radius of the circle that passes
    through these points

    Based on: https://math.stackexchange.com/questions/133638/
    how-does-this-equation-to-find-the-radius-from-3-points-actually-work

    Args:
        points (np.ndarray): The points for which should be used to calculate the radius

    Returns:
        np.ndarray: The calculated radius
    """
    # implements the equation discussed here:
    #
    # assert points.shape[-2:] == (3, 2)
    # get side lengths
    pointsCircular = points[..., [0, 1, 2, 0], :]
    lenSides = traceDistanceToNext(pointsCircular)

    # calc prod of sides
    prodOfSides = np.prod(lenSides, axis=-1, keepdims=True)

    # calc area of triangle
    # https://www.mathopenref.com/heronsformula.html

    # calc half of perimeter
    perimeter = np.sum(lenSides, axis=-1, keepdims=True)
    halfPerimeter = perimeter / 2
    halfPerimeterMinusSides = halfPerimeter - lenSides
    areaSqr = (
        np.prod(halfPerimeterMinusSides, axis=-1, keepdims=True) * halfPerimeter
    )
    area = np.sqrt(areaSqr)

    radius = prodOfSides / (area * 4)

    radius = radius[..., 0]
    return radius


Numeric = TypeVar("Numeric", float, np.ndarray)


def linearlyCombineValuesOverTime(
    tee: float, deltaTime: float, previousValue: Numeric, newValue: Numeric
) -> Numeric:
    """
    Linear combination of two values over time
    (see https://de.wikipedia.org/wiki/PT1-Glied)
    Args:
        tee (float): The parameter selecting how much we keep from the previous value
        and how much we update from the new
        delta_time (float): The time difference between the previous and new value
        previous_value (Numeric): The previous value
        new_value (Numeric): The next value

    Returns:
        Numeric: The combined value
    """
    teeStar = 1 / (tee / deltaTime + 1)
    combinedValue: Numeric = teeStar * (newValue - previousValue) + previousValue
    return combinedValue


def oddSquare(values: Numeric) -> Numeric:
    """
    Squares the input value while preserving its sign and returns the result.

    Args:
        values (Numeric): The input numeric value.

    Returns:
        Numeric: The square of the input value with the same sign.
    """
    return cast(Numeric, np.sign(values) * np.square(values))


def eulerAnglesToQuaternion(eulerAngles: np.ndarray) -> np.ndarray:
    """
    Converts Euler angles to a quaternion representation.

    Args:
        euler_angles (np.ndarray): Euler angles as an [...,3] array. Order is
        [roll, pitch, yaw]

    Returns:
        np.ndarray: The quaternion representation in [..., 4] [x, y, z, w] order
    """
    rollIndex, pitchIndex, yawIndex = 0, 1, 2
    sinValues = np.sin(eulerAngles * 0.5)
    cosValues = np.cos(eulerAngles * 0.5)

    cosYaw = cosValues[..., yawIndex]
    sinYaw = sinValues[..., yawIndex]
    cosYitch = cosValues[..., pitchIndex]
    sinPitch = sinValues[..., pitchIndex]
    cosRoll = cosValues[..., rollIndex]
    sinRoll = sinValues[..., rollIndex]

    quaternionX = sinRoll * cosYitch * cosYaw - cosRoll * sinPitch * sinYaw
    quaternionY = cosRoll * sinPitch * cosYaw + sinRoll * cosYitch * sinYaw
    quaternionZ = cosRoll * cosYitch * sinYaw - sinRoll * sinPitch * cosYaw
    quaternionW = cosRoll * cosYitch * cosYaw + sinRoll * sinPitch * sinYaw

    returnValue = np.stack(
        [quaternionX, quaternionY, quaternionZ, quaternionW], axis=-1
    )
    return returnValue


def quaternionToEulerAngles(quaternion: np.ndarray) -> np.ndarray:
    """
    Converts a quaternion to Euler angles. Based on
    https://stackoverflow.com/a/37560411.

    Args:
        quaternion (np.ndarray): The quaternion as an [..., 4] array. Order is
        [x, y, z, w]

    Returns:
        np.ndarray: The Euler angles as an [..., 3] array. Order is [roll, pitch, yaw]
    """
    xIndex, yIndex, zIndex, wIndex = 0, 1, 2, 3
    xValue = quaternion[..., xIndex]
    yValue = quaternion[..., yIndex]
    zValue = quaternion[..., zIndex]
    wValue = quaternion[..., wIndex]

    ySquare = yValue * yValue
    temporary0 = -2.0 * (ySquare + zValue * zValue) + 1.0
    temporary1 = +2.0 * (xValue * yValue + wValue * zValue)
    temporary2 = -2.0 * (xValue * zValue - wValue * yValue)
    temporary3 = +2.0 * (yValue * zValue + wValue * xValue)
    temporary4 = -2.0 * (xValue * xValue + ySquare) + 1.0

    temporary2 = np.clip(temporary2, -1.0, 1.0)

    roll = np.arctan2(temporary3, temporary4)
    pitch = np.arcsin(temporary2)
    yaw = np.arctan2(temporary1, temporary0)

    returnValue = np.stack([roll, pitch, yaw], axis=-1)
    return returnValue


@myNjit
def pointsInsideEllipse(
    points: np.ndarray,
    center: np.ndarray,
    majorDirection: np.ndarray,
    majorRadius: float,
    minorRadius: float,
) -> np.ndarray:
    """
    Checks if a set of points are inside an ellipse.

    Args:
        points: The points as an [..., 2] array.
        center: The center of the ellipse as an [2] array.
        major_direction: The major direction of the ellipse as an [2] array.
        major_radius: The major radius of the ellipse.
        minor_radius: The minor radius of the ellipse.

    Returns:
        An [...] array of booleans.
    """

    # Center the points around the center
    # [..., 2]
    centeredPoints = points - center
    # Calculate angle of the major direction with the x-axis
    # [1]
    majorDirectionAngle = np.arctan2(majorDirection[1], majorDirection[0])
    # Rotate the points around the center of the ellipse
    # [..., 2]
    rotatedPoints = rotate(centeredPoints, -majorDirectionAngle)
    # [2]
    radiiSquare = np.array([majorRadius, minorRadius]) ** 2
    # [...]    [..., 2]              [2]
    criterionValue = (rotatedPoints**2 / radiiSquare).sum(axis=-1)

    maskIsInside = criterionValue < 1
    return maskIsInside


def centerOfCircleFrom3Points(
    point1: np.ndarray,
    point2: np.ndarray,
    point3: np.ndarray,
    atol: float = 1e-6,
) -> np.ndarray:
    """
    Calculates the center of a circle from three points.

    Adapted from http://paulbourke.net/geometry/circlesphere/Circle.cpp (CalcCircle)

    Args:
        point_1: The first point as an [2] array.
        point_2: The second point as an [2] array.
        point_3: The third point as an [2] array.

    Returns:
        The center of the circle as an [2] array.
    """
    yDelta1 = point2[1] - point1[1]
    xDelta1 = point2[0] - point1[0]
    yDelta2 = point3[1] - point2[1]
    xDelta2 = point3[0] - point2[0]

    if np.isclose(xDelta1, 0.0, atol=atol) and np.isclose(xDelta2, 0.0, atol=atol):
        centerX = (point2[0] + point3[0]) / 2
        centerY = (point1[1] + point2[1]) / 2
        return np.array([centerX, centerY])  # early return

    slope1 = yDelta1 / xDelta1
    slope2 = yDelta2 / xDelta2
    if np.isclose(slope1, slope2, atol=atol):
        raise ValueError("Points are colinear")

    centerX = (
        slope1 * slope2 * (point1[1] - point3[1])
        + slope2 * (point1[0] + point2[0])
        - slope1 * (point2[0] + point3[0])
    ) / (2 * (slope2 - slope1))

    centerY = (
        -(centerX - (point1[0] + point2[0]) / 2) / slope1
        + (point1[1] + point2[1]) / 2
    )

    center = np.array([centerX, centerY])
    return center


@myNjit
def circleFit(coords: np.ndarray, maxIter: int = 99) -> np.ndarray:
    """
    Fit a circle to a set of points. This function is adapted from the hyper_fit function
    in the circle-fit package (https://pypi.org/project/circle-fit/). The function is
    a njit version of the original function with some input validation removed. Furthermore,
    the residuals are not calculated or returned.

    Args:
        coords: The coordinates of the points as an [N, 2] array.
        max_iter: The maximum number of iterations.

    Returns:
        An array with 3 elements:
        - center x
        - center y
        - radius
    """

    x0 = coords[:, 0]
    y0 = coords[:, 1]

    n = x0.shape[0]

    xi = x0 - x0.mean()
    yi = y0 - y0.mean()
    zi = xi * xi + yi * yi

    # compute moments
    mxy = (xi * yi).sum() / n
    mxx = (xi * xi).sum() / n
    myy = (yi * yi).sum() / n
    mxz = (xi * zi).sum() / n
    myz = (yi * zi).sum() / n
    mzz = (zi * zi).sum() / n

    # computing the coefficients of characteristic polynomial
    mz = mxx + myy
    covXY = mxx * myy - mxy * mxy
    varZ = mzz - mz * mz

    a2 = 4 * covXY - 3 * mz * mz - mzz
    a1 = varZ * mz + 4.0 * covXY * mz - mxz * mxz - myz * myz
    a0 = mxz * (mxz * myy - myz * mxy) + myz * (myz * mxx - mxz * mxy) - varZ * covXY
    a22 = a2 + a2

    # finding the root of the characteristic polynomial
    y = a0
    x = 0.0
    for _ in range(maxIter):
        dy = a1 + x * (a22 + 16.0 * x * x)
        xNew = x - y / dy
        if xNew == x or not np.isfinite(xNew):
            break
        yNew = a0 + xNew * (a1 + xNew * (a2 + 4.0 * xNew * xNew))
        if abs(yNew) >= abs(y):
            break
        x, y = xNew, yNew

    det = x * x - x * mz + covXY
    xCenter = (mxz * (myy - x) - myz * mxy) / det / 2.0
    yCenter = (myz * (mxx - x) - mxz * mxy) / det / 2.0

    x = xCenter + x0.mean()
    y = yCenter + y0.mean()
    r = np.sqrt(abs(xCenter**2 + yCenter**2 + mz))

    return np.array([x, y, r])


if __name__ == "__main__":
    p1, p2, p3 = unit2dVectorFromAngle(np.array([0, 0.3, 0.31]))
    print(centerOfCircleFrom3Points(p1, p2, p3))

    p1, p2, p3 = unit2dVectorFromAngle(np.array([0, 0.8, 4])) + 10
    print(centerOfCircleFrom3Points(p1, p2, p3))

    p1, p2, p3 = np.array([-1, 0.0]), np.array([0, 1.0]), np.array([1.0, 0.0])
    print(centerOfCircleFrom3Points(p1, p2, p3))

    p1, p2, p3 = np.array([0, 0.0]), np.array([0, 1.0]), np.array([0.0, 2.0])
    print(centerOfCircleFrom3Points(p1, p2, p3))


# @my_njit
def angleDifference(angle1: np.ndarray, angle2: np.ndarray) -> np.ndarray:
    """
    Calculate the difference between two angles. The range of the difference is [-pi, pi].
    The order of the angles *is* important.

    Args:
        angle1: First angle.
        angle2: Second angle.

    Returns:
        The difference between the two angles.
    """
    return (angle1 - angle2 + 3 * np.pi) % (2 * np.pi) - np.pi  # type: ignore
