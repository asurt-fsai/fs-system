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
