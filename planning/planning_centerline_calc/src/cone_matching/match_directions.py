"""
Description: Calculate the search direction to find a match for each cone.
"""

import numpy as np

from src.utils.cone_types import ConeTypes
from src.utils.math_utils import myNjit, rotate
from src.types_file.types import FloatArray


@myNjit
def calculateSearchDirectionForOne(
    cones: FloatArray,
    idxs: FloatArray,
    coneType: ConeTypes
) -> FloatArray:
    """
    Calculates the search direction for one cone
    """
    assert len(idxs) == 2

    trackDirection = cones[idxs[1]] - cones[idxs[0]]

    rotationAngle = np.pi / 2 if coneType == ConeTypes.right else -np.pi / 2

    searchDirection = rotate(trackDirection, rotationAngle)

    result: FloatArray = searchDirection / np.linalg.norm(searchDirection)

    return result


# @my_njit
def calculateMatchSearchDirection(
    cones: FloatArray,
    coneType: ConeTypes,
) -> FloatArray:
    """
    Calculates the search direction for all cones.
    """
    numberOfCones = len(cones)
    assert numberOfCones > 1

    conesXY = cones[:, :2]

    out = np.zeros((numberOfCones, 2))
    out[0] = calculateSearchDirectionForOne(conesXY, np.array([0, 1]), coneType)
    out[-1] = calculateSearchDirectionForOne(conesXY, np.array([-2, -1]), coneType)

    for i in range(1, numberOfCones - 1):
        out[i] = calculateSearchDirectionForOne(conesXY, np.array([i - 1, i + 1]), coneType)

    return out
