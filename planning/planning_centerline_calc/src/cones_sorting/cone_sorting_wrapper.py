"""
Cone sorting wrapper class
Description: Entry point for Pathing/ConeSorting
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from src.cones_sorting.core_cone_sorter import ConeSorter
from src.types_file.types import FloatArray
from src.utils.cone_types import ConeTypes


class ConeSorting:
    """Class that takes all Pathing/ConeSorting responsibilities"""

    def __init__(
        self,
        maxNNeighbors: int,
        maxDist: float,
        maxDistToFirst: float,
        maxLength: int,
        thresholdDirectionalAngle: float,
        thresholdAbsoluteAngle: float,
    ):
        """
        Init method.

        Args:
            maxNNeighbors, maxDist, maxDistToFirst: Arguments for ConeSorter.
            maxLength: Arguments for ConeSorter. The maximum length of the valid trace
                in a sorting algorithm.
            thresholdDirectionalAngle: The threshold for the directional angle that is
                the minimum angle for consecutive cones to be connected in the direction
                of the trace (clockwise for left cones, counter-clockwise for right cones).
            thresholdAbsoluteAngle: The threshold for the absolute angle that is the
                minimum angle for consecutive cones to be connected regardless of the
                cone type.
            useUnknownCones: Whether to use unknown ( as is no color info is known) cones
                in the sorting algorithm.
        """
        self.newInput = ConeSortingInput()

        self.const = ConeSortingConstants(
            maxNNeighbors=maxNNeighbors,
            maxDist=maxDist,
            maxDistToFirst=maxDistToFirst,
            maxLength=maxLength,
            thresholdDirectionalAngle=thresholdDirectionalAngle,
            thresholdAbsoluteAngle=thresholdAbsoluteAngle,
        )
        self.state = ConeSortingState()

    def setNewInput(self, newInput: ConeSortingInput) -> None:
        """Save inputs from other nodes in a varible."""
        self.newInput = newInput

    def transitionInputToState(self) -> None:
        """Parse and save the inputs in the state varible."""
        self.state.positionGlobal, self.state.directionGlobal = (
            self.newInput.slamPosition,
            self.newInput.slamDirection,
        )

        self.state.conesByTypeArray = self.newInput.perceptionCones.copy()
        self.state.conesByTypeArray[ConeTypes.UNKNOWN] = np.zeros((0, 2))

    def runConeSorting(
        self,
    ) -> Tuple[FloatArray, FloatArray]:
        """
        Calculate the sorted cones.

        Returns:
            The sorted cones. The first array contains the sorted blue (left) cones and
            the second array contains the sorted yellow (right) cones.
        """
        # make transition from set inputs to usable state varibles
        self.transitionInputToState()

        # calculate the sorted cones
        coneSorter = ConeSorter(
            self.const.maxNNeighbors,
            self.const.maxDist,
            self.const.maxDistToFirst,
            self.const.maxLength,
            self.const.thresholdDirectionalAngle,
            self.const.thresholdAbsoluteAngle,
        )

        leftCones, rightCones = coneSorter.sortLeftRight(
            self.state.conesByTypeArray,
            self.state.positionGlobal,
            self.state.directionGlobal,
        )

        return leftCones, rightCones


@dataclass
class ConeSortingInput:
    """Dataclass holding inputs."""

    perceptionCones: list[FloatArray] = field(
        default_factory=lambda: [np.zeros((0, 2)) for _ in ConeTypes]
    )
    slamPosition: FloatArray = field(default_factory=lambda: np.zeros(2))
    slamDirection: np.float_ = np.float_(0.0)


@dataclass
class ConeSortingConstants:
    """Dataclass holding calculation parameters"""

    thresholdDirectionalAngle: float
    thresholdAbsoluteAngle: float
    maxNNeighbors: int
    maxDist: float
    maxDistToFirst: float
    maxLength: int


@dataclass
class ConeSortingState:
    """Dataclass holding calculation variables."""

    positionGlobal: FloatArray = field(default_factory=lambda: np.zeros(2))
    directionGlobal: np.float_ = np.float_(0.0)
    conesByTypeArray: list[FloatArray] = field(
        default_factory=lambda: [np.zeros((0, 3)) for _ in ConeTypes]
    )
