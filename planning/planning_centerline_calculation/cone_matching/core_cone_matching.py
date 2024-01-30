#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Cone matching class.

Description: Provides class interface to functional cone matching.
Project: fsd_path_planning
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np
from icecream import ic  # pylint: disable=unused-import

from planning_centerline_calculation.cone_matching.functional_cone_matching import (
    calculateVirtualConesForBothSides,
)
from planning_centerline_calculation.types import FloatArray, IntArray
from planning_centerline_calculation.utils.cone_types import ConeTypes

MatchedCones = Tuple[FloatArray, FloatArray, IntArray, IntArray]


@dataclass
class ConeMatchingInput:
    """Dataclass holding inputs."""

    sortedCones: list[FloatArray] = field(
        default_factory=lambda: [np.zeros((0, 2)) for _ in ConeTypes]
    )
    slamPosition: FloatArray = field(default_factory=lambda: np.zeros((2)))
    slamDirection: FloatArray = field(default_factory=lambda: np.zeros((2)))


@dataclass
class ConeMatchingState:
    """Dataclass holding calculation variables."""

    minTrackWidth: float
    maxSearchRange: float
    maxSearchAngle: float
    matchesShouldBeMonotonic: bool
    sortedLeft: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    sortedRight: FloatArray = field(default_factory=lambda: np.zeros((0, 2)))
    positionGlobal: FloatArray = field(init=False)
    directionGlobal: FloatArray = field(init=False)


class ConeMatching:
    """Class that takes all cone matching and virtual cone responsibilities."""

    def __init__(
        self,
        minTrackWidth: float,
        maxSearchRange: float,
        maxSearchAngle: float,
        matchesShouldBeMonotonic: bool,
    ):
        """
        Init method.

        Args:
        """

        self.input = ConeMatchingInput()
        self.state = ConeMatchingState(
            minTrackWidth=minTrackWidth,
            maxSearchRange=maxSearchRange,
            maxSearchAngle=maxSearchAngle,
            matchesShouldBeMonotonic=matchesShouldBeMonotonic,
        )

    def setNewInput(self, coneMatchingInput: ConeMatchingInput) -> None:
        """Save inputs from other software nodes in variable."""
        self.input = coneMatchingInput

    def transitionInputToState(self) -> None:
        """Parse and save the inputs in state variable."""
        self.state.positionGlobal, self.state.directionGlobal = (
            self.input.slamPosition,
            self.input.slamDirection,
        )

        self.state.sortedLeft = self.input.sortedCones[ConeTypes.LEFT]
        self.state.sortedRight = self.input.sortedCones[ConeTypes.RIGHT]

    def runConeMatching(self) -> MatchedCones:
        """
        Calculate matched cones.

        Returns:
            Matched cones.
                The left cones with virtual cones.
                The right cones with virtual cones.
                The indices of the matches of the right cones for each left cone.
                The indices of the matches of the left cones for each right cone.

        """
        self.transitionInputToState()

        majorRadius = self.state.maxSearchRange * 1.5
        minorRadius = self.state.minTrackWidth

        (
            (leftConesWithVirtual, _, leftToRightMatch),
            (rightConesWithVirtual, _, rightToLeftMatch),
        ) = calculateVirtualConesForBothSides(
            self.state.sortedLeft,
            self.state.sortedRight,
            self.state.positionGlobal,
            self.state.directionGlobal,
            self.state.minTrackWidth,
            majorRadius,
            minorRadius,
            self.state.maxSearchAngle,
            self.state.matchesShouldBeMonotonic,
        )

        return (
            leftConesWithVirtual,
            rightConesWithVirtual,
            leftToRightMatch,
            rightToLeftMatch,
        )
    