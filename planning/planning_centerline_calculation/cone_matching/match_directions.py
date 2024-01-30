import numpy as np

from planning_centerline_calculation.utils.cone_types import ConeTypes
from planning_centerline_calculation.utils.math_utils import myNjit, rotate


@myNjit
def calculateSearchDirectionForOne(cones, idxs, cone_type):
    """
    Calculates the search direction for one cone
    """
    assert len(idxs) == 2

    trackDirection = cones[idxs[1]] - cones[idxs[0]]

    rotationAngle = np.pi / 2 if cone_type == ConeTypes.RIGHT else -np.pi / 2

    searchDirection = rotate(trackDirection, rotationAngle)

    return searchDirection / np.linalg.norm(searchDirection)


#@my_njit
def calculateMatchSearchDirection(
    cones,
    coneType: ConeTypes,
):
    numberOfCones = len(cones)
    assert numberOfCones > 1

    conesXY = cones[:, :2]

    out = np.zeros((numberOfCones, 2))
    out[0] = calculateSearchDirectionForOne(conesXY, np.array([0, 1]), coneType)
    out[-1] = calculateSearchDirectionForOne(
        conesXY, np.array([-2, -1]), coneType
    )

    for i in range(1, numberOfCones - 1):
        out[i] = calculateSearchDirectionForOne(
            conesXY, np.array([i - 1, i + 1]), coneType
        )

    return out
