"""
Description: Enum for cone types (yellow, blue, etc...)
"""
from enum import IntEnum


class ConeTypes(IntEnum):
    """
    Enum for all possible cone types
    """

    UNKNOWN = 0
    RIGHT = YELLOW = 1
    LEFT = BLUE = 2
    START_FINISH_AREA = ORANGE_SMALL = 3
    START_FINISH_LINE = ORANGE_BIG = 4


def invertConeType(coneType: ConeTypes) -> ConeTypes:
    """
    Inverts the cone type. E.g. LEFT -> RIGHT
    Args:
        cone_type: The cone type to invert
    Returns:
        ConeTypes: The inverted cone type
    """
    if coneType == ConeTypes.LEFT:
        return ConeTypes.RIGHT
    if coneType == ConeTypes.RIGHT:
        return ConeTypes.LEFT
    return coneType