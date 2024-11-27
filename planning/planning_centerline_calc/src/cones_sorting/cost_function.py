"""
Description: This File calculates the costs for the different path versions
"""
import numpy as np

from src.cones_sorting.nearby_cone_search import numberConesOnEachSideForEachConfig
from src.types_file.types import BoolArray, FloatArray, IntArray
from src.utils.cone_types import ConeTypes
from src.utils.math_utils import (
    angleDifference,
    vecAngleBetween,
    traceDistanceToNext,
    unit2dVectorFromAngle,
)


def costConfigurations(
    points: FloatArray,
    configurations: IntArray,
    coneType: ConeTypes,
    vehicleDirection: np.float_,
    *,
    returnIndividualCosts: bool,
) -> FloatArray:
    """
    Calculates the cost for each provided configuration
    Args:
        points: The underlying points
        configurations: An array of indicies defining the configurations of the provided points
        coneType: The type of the cone (left/right)
    Returns:
        The cost for each configuration
    """
    pointsXY = points[:, :2]

    if len(configurations) == 0:
        return np.zeros(0)

    angleCost = calcAngleCostForConfigurations(pointsXY, configurations)
    # maximum allowed distance between cones is 5 meters
    residualDistanceCost = calcDistanceCost(pointsXY, configurations, 3)

    numberOfConesCost = calcNumberOfConesCost(configurations)

    initialDirectionCost = calcInitialDirectionCost(pointsXY, configurations, vehicleDirection)

    changeOfDirectionCost = calcChangeOfDirectionCost(pointsXY, configurations)

    conesOnEitherCost = calcConesOnEitherCost(pointsXY, configurations, coneType)

    wrongDirectionCost = calcWrongDirectionCost(pointsXY, configurations, coneType)

    factors: FloatArray = np.array(
        [1000.0, 200.0, 5000.0, 1000.0, 0.0, 1000.0, 1000.0]
    )  # [1000.0, 200.0, 5000.0, 1000.0, 0.0, 1000.0, 1000.0]
    factors = factors / factors.sum()
    finalCosts = (
        np.column_stack(
            [
                angleCost,
                residualDistanceCost,
                numberOfConesCost,
                initialDirectionCost,
                changeOfDirectionCost,
                conesOnEitherCost,
                wrongDirectionCost,
            ]
        )
        * factors
    )

    if returnIndividualCosts:
        return finalCosts.astype(float)

    return finalCosts.sum(axis=-1)  # type: ignore


def calcAngleCostForConfigurations(
    points: FloatArray,
    configurations: IntArray,
) -> FloatArray:
    """
    Calculates the angle cost for the provided configurations
    given a set of points and many index lists defining the configurations
    Args:
        points: The points to be used
        configurations: An array of indicies defining the configurations
    Returns:
        np.array: The score of each configuration
    """

    angles = calcAngleToNext(points, configurations)

    isPartOfConfiguration = (configurations != -1)[:, 2:]

    # invert angles and normalize to 0-1
    anglesAsCost = (np.pi - angles) / np.pi

    anglesAsCostFiltered = anglesAsCost * isPartOfConfiguration

    anglesAreUnderThreshold = np.logical_and(angles < np.deg2rad(40), isPartOfConfiguration)

    # we will multiply the score by the number of angles that are under the threshold
    costFactors = anglesAreUnderThreshold.sum(axis=-1) + 1

    # get sum of costs
    costs: FloatArray = anglesAsCostFiltered.sum(axis=-1) / isPartOfConfiguration.sum(axis=-1)

    costs = costs * costFactors
    return costs


def calcAngleToNext(points: FloatArray, configurations: IntArray) -> FloatArray:
    """
    Calculates the angle from one cone the previous
    and the next one for all the provided configurations
    """
    allToNext = getConfigurationsDiff(points, configurations)

    maskShouldOverwrite = (configurations == -1)[:, 1:]
    allToNext[maskShouldOverwrite] = 100

    fromMiddleToNext = allToNext[..., 1:, :]
    fromPrevToMiddle = allToNext[..., :-1, :]
    fromMiddleToPrev = -fromPrevToMiddle

    angles = vecAngleBetween(fromMiddleToNext, fromMiddleToPrev)
    return angles


def getConfigurationsDiff(points: FloatArray, configurations: IntArray) -> FloatArray:
    """
    Gets the difference from each point to its next for each order defined by the configurations
    Args:
        points: The points for which the diffrences should be calculated
        configurations: (n,m), all the configurations that define the orders
    Returns:
        np.array: The differences from one point to the next for each configuration
    """
    results: FloatArray
    results = points[configurations[..., :-1]]
    results -= points[configurations[..., 1:]]
    return results


def calcDistanceCost(
    points: FloatArray, configurations: IntArray, thresholdDistance: float
) -> FloatArray:
    """
    Calculates the sum of residual distances between consecutive cones. The residual
    distance is defined as the distance between two cones that is over 'thresholdDistance'.
    If two cones have a distance less than 'thresholdDistance', the residual distance is 0.
    """
    pointsInConfigurations = points[configurations]
    distancesToNext = traceDistanceToNext(pointsInConfigurations)

    distancesToNextFiltered = distancesToNext * (configurations != -1)[:, 1:]

    residualDistance = np.maximum(0, distancesToNextFiltered - thresholdDistance)
    sumOfResidualDistancesForConfigurations: FloatArray = residualDistance.sum(axis=-1)
    return sumOfResidualDistancesForConfigurations


def calcNumberOfConesCost(configurations: IntArray) -> FloatArray:
    """
    Calculates the number of cones in each configuration
    Args:
        configurations: An array of indicies defining the configurations of the provided points
    Returns:
        A cost for each configuration
    """
    mask: BoolArray = configurations != -1
    numberOfCones: IntArray = mask.sum(axis=-1)
    # we prefer longer configurations
    cost = 1 / numberOfCones
    return cost


def calcInitialDirectionCost(
    points: FloatArray, configurations: IntArray, vehicleDirection: np.float_
) -> FloatArray:
    """
    Calculates the initial direction cost between points and vehicle direction.

    Args:
        points (FloatArray): Array of points.
        configurations (IntArray): Array of configurations.
        vehicleDirection (FloatArray): Array representing the vehicle direction.

    Returns:
        FloatArray: Array of initial direction costs.
    """
    vehicleDirectionArray = unit2dVectorFromAngle(np.array(vehicleDirection))
    pointsConfigFirstTwo = np.diff(points[configurations][:, :2], axis=1)[:, 0]

    return vecAngleBetween(pointsConfigFirstTwo, vehicleDirectionArray)


def calcChangeOfDirectionCost(points: FloatArray, configurations: IntArray) -> FloatArray:
    """
    Calculates the change of direction cost in each configuration.
    This is done for each configuration using the following steps:
    1. Calculate the empiric first derivative of the configuration
    2. Calculate the angle of the first derivative
    3. Calculate the zero crossings of the angle along the configuration
    4. Calculate the sum of changes in the angle between the zero crossings

    Args:
        points: The underlying points
        configurations: An array of indicies defining the configurations of the provided points
    Returns:
        A cost for each configuration
    """
    out = np.zeros(configurations.shape[0])
    for i, config in enumerate(configurations):
        config = config[config != -1]
        if len(config) == 3:
            continue

        pointsOfConfiguration = points[config]

        diff1 = pointsOfConfiguration[1:] - pointsOfConfiguration[:-1]

        diff1 = np.diff(pointsOfConfiguration, axis=0)
        angle = np.arctan2(diff1[:, 1], diff1[:, 0])
        diffrence = angleDifference(angle[:-1], angle[1:])

        maskZeroCrossings = np.sign(diffrence[:-1]) != np.sign(diffrence[1:])
        rawCostValues = np.abs(diffrence[:-1] - diffrence[1:])

        costValues = rawCostValues * maskZeroCrossings
        out[i] = np.sum(costValues)
    return out


def calcConesOnEitherCost(
    points: FloatArray, configurations: IntArray, coneType: ConeTypes
) -> FloatArray:
    """
    Calculates the cost function for cones on either side.

    Args:
        points (FloatArray): Array of points.
        configurations (IntArray): Array of configurations.
        coneType (SortableConeTypes): Type of cone.

    Returns:
        FloatArray: Array of calculated cost values.
    """
    nGood, nBad = numberConesOnEachSideForEachConfig(
        points,
        configurations,
        coneType,
        6.0,
        np.pi / 1.5,
    )
    print(nGood)
    print(nBad)
    diff = nGood - nBad
    mValue = diff.min()

    diff += np.abs(mValue) + 1

    return 1 / diff


def calcWrongDirectionCost(
    points: FloatArray, configurations: IntArray, coneType: ConeTypes
) -> FloatArray:
    """
    Args:
        points: The underlying points
        configurations: An array of indicies defining the configurations of the provided points
    Returns:
        A cost for each configuration
    """
    out = np.zeros(configurations.shape[0])

    unwantedDirectionSign = 1 if coneType == ConeTypes.left else -1

    for i, config in enumerate(configurations):
        config = config[config != -1]
        if len(config) == 3:
            continue

        pointsOfConfiguration = points[config]

        diff1 = pointsOfConfiguration[1:] - pointsOfConfiguration[:-1]

        diff1 = np.diff(pointsOfConfiguration, axis=0)
        angle = np.arctan2(diff1[:, 1], diff1[:, 0])
        diffrence = angleDifference(angle[:-1], angle[1:])

        maskWrongDirection = np.sign(diffrence) == unwantedDirectionSign
        maskThreshold = np.abs(diffrence) > np.deg2rad(40)

        mask = maskWrongDirection & maskThreshold

        costValues = np.abs(diffrence[mask].sum())

        out[i] = costValues
    # print(out)
    return out
