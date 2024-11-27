"""
Description: Enum for cone types (yellow, blue, etc...)
"""
from enum import IntEnum


class ConeTypes(IntEnum):
    """
    Enum for all possible cone types
    """

    UNKNOWN = 0
    right = YELLOW = 1
    left = BLUE = 2
    startFinishArea = ORANGE_SMALL = 3
    startFinishLine = ORANGE_BIG = 4


def invertConeType(coneType: ConeTypes) -> ConeTypes:
    """
    Inverts the cone type. E.g. LEFT -> RIGHT
    Args:
        cone_type: The cone type to invert
    Returns:
        ConeTypes: The inverted cone type
    """
    if coneType == ConeTypes.left:
        return ConeTypes.right
    if coneType == ConeTypes.right:
        return ConeTypes.left
    return coneType
