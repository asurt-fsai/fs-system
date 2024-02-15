"""
Cone sorting wrapper class
Description: Entry point for Pathing/ConeSorting
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from planning_centerline_calculation.cones_sorting.core_cone_sorter import ConeSorter
from planning_centerline_calculation.types import FloatArray
from planning_centerline_calculation.utils.cone_types import ConeTypes


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
            useUnknownCones: bool,
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
        self.input = ConeSortingInput()

        self.state = ConeSortingState(
            maxNNeighbors=maxNNeighbors,
            maxDist=maxDist,
            maxDistToFirst=maxDistToFirst,
            maxLength=maxLength,
            thresholdDirectionalAngle=thresholdDirectionalAngle,
            thresholdAbsoluteAngle=thresholdAbsoluteAngle,
            useUnknownCones=useUnknownCones,
        )

    def setNewInput(self, input: ConeSortingInput) -> None:
        """Save inputs from other nodes in a varible."""
        self.input = input

    def transitionInputToState(self) -> None:
        """Parse and save the inputs in the state varible."""
        self.state.positionGlobal, self.state.directionGlobal = (
            self.input.slamPosition,
            self.input.slamDirection,
        )

        self.state.conesByTypeArray = self.input.perceptionCones.copy()
        if not self.state.useUnknownCones:
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
            self.state.maxNNeighbors,
            self.state.maxDist,
            self.state.maxDistToFirst,
            self.state.maxLength,
            self.state.thresholdDirectionalAngle,
            self.state.thresholdAbsoluteAngle,
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
    slamDirection: FloatArray = field(default_factory=lambda: np.zeros(2))

@dataclass
class ConeSortingState:
    """Dataclass holding calculation variables."""

    thresholdDirectionalAngle: float
    thresholdAbsoluteAngle: float
    maxNNeighbors: int
    maxDist: float
    maxDistToFirst: float
    maxLength: int
    useUnknownCones: bool
    positionGlobal: FloatArray = field(default_factory=lambda: np.zeros(2))
    directionGlobal: FloatArray = field(default_factory=lambda: np.array([0, 1.0]))
    conesByTypeArray: list[FloatArray] = field(
        default_factory=lambda: [np.zeros((0, 3)) for _ in ConeTypes]
    )
