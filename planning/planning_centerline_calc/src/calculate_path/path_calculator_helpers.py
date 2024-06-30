#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Core path calculation.

Description: A module for path update calculation that will be used in combination with
the existing path

"""
import numpy as np
from icecream import ic  # pylint: disable=unused-import
from typing_extensions import Literal

from src.types_file.types import FloatArray
from src.utils.cone_types import ConeTypes
from src.utils.math_utils import rotate, unit2dVectorFromAngle

ConeTypesForPathCalculation = Literal[ConeTypes.left, ConeTypes.right]
HALF_PI = np.pi / 2

class PathCalculatorHelpers:
    """A class for calculating the update path that will be combined with the existing path."""

    def calculateChordPath(
        self, radius: float, maximumAngle: float, numberOfPoints: int
    ) -> FloatArray:
        """
        Calculate a chord (part of circle) path with a specific radius.

        Args:
            radius: The radius of the chord path
            maximum_angle: The angle of the chord
            number_of_points: The number of points the path should be evaluated
            for
        Returns:
            The arc path
        """

        points: FloatArray = (
            # create points on a circle
            unit2dVectorFromAngle(np.linspace(0, np.abs(maximumAngle), numberOfPoints))
        )
        # rotate so initial points, point to the right
        pointsCentered: FloatArray = points - [1, 0]  # bring x axis to center
        pointsCenteredScaled: FloatArray = pointsCentered * radius  # scale
        pointsCenteredScaledRotated = rotate(pointsCenteredScaled, -HALF_PI)

        # handle negative angles
        pointsCenteredScaledRotated[:, 1] *= np.sign(maximumAngle)
        return pointsCenteredScaledRotated

    def calculateAlmostStraightPath(self) -> FloatArray:
        """
        Calculate a chord path with a very high radius and a very small chord angle.

        Returns:
            np.ndarray: The straight-like chord path update
        """
        return self.calculateChordPath(
            # values for a slightly circular path to the right
            radius=1000,
            maximumAngle=np.pi / 50,
            numberOfPoints=40,
        )
