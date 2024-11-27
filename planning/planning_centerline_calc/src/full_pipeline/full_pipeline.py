#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Description: A class that runs the whole path planning pipeline.

- Cone sorting
- Cone Matching
- Path Calculation

"""
from __future__ import annotations

from typing import List, Optional
from dataclasses import dataclass
import numpy as np

from src.cones_sorting.cone_sorting_wrapper import ConeSortingInput, ConeSorting
from src.cone_matching.core_cone_matching import ConeMatching, ConeMatchingInput
from src.calculate_path.core_calculate_path import CalculatePath, PathCalculationInput
from src.corner_case.corner_case_path import CornerCasesPath
from src.types_file import FloatArray

from src.utils.cone_types import ConeTypes


@dataclass
class ParametersState: # pylint: disable=too-many-instance-attributes
    """Dataclass holding parameters variables."""
    thresholdDirectionalAngle: float = np.deg2rad(40)
    thresholdAbsoluteAngle: float = np.deg2rad(65)
    maxNNeighbors: int = 5
    maxDist: float = 7
    maxDistToFirst: float = 10.0
    maxLength: int = 7

    minTrackWidth: float = 3
    maxSearchRange: float = 5
    maxSearchAngle: float = np.deg2rad(50)
    matchesShouldBeMonotonic: bool = True

    maximalDistanceForValidPath: float = 5
    mpcPathLength: float = 20
    mpcPredictionHorizon: int = 40


class PathPlanner:
    """
    This class is responsible for planning a path for a vehicle between cones.

    It utilizes three sub-components to achieve this:

    1. ConeSorting: Sorts the cones based on their relative position to the vehicle.
    2. ConeMatching: Matches cones from left and right sides based on their positions.
    3. CalculatePath: Calculates the optimal path for the vehicle considering
                       sorted and matched cones, as well as a potentially provided global path.

    Attributes:
        coneSorting: An instance of the ConeSorting class used for cone sorting.
        coneMatching: An instance of the ConeMatching class for cone matching.
        calculatePath: An instance of the CalculatePath class for path calculation.
        globalPath: A global path to be considered during path planning (Optional).
    """

    def __init__(self, params: ParametersState) -> None:
        self.coneSorting = ConeSorting(
            maxNNeighbors=params.maxNNeighbors,
            maxDist=params.maxDist,
            maxDistToFirst=params.maxDistToFirst,
            maxLength=params.maxLength,
            thresholdDirectionalAngle=params.thresholdDirectionalAngle,
            thresholdAbsoluteAngle=params.thresholdAbsoluteAngle,
        )
        self.coneMatching = ConeMatching(
            minTrackWidth=params.minTrackWidth,
            maxSearchRange=params.maxSearchRange,
            maxSearchAngle=params.maxSearchAngle,
            matchesShouldBeMonotonic=params.matchesShouldBeMonotonic,
        )
        self.calculatePath = CalculatePath(
            maximalDistanceForValidPath=params.maximalDistanceForValidPath,
            mpcPathLength=params.mpcPathLength,
            mpcPredictionHorizon=params.mpcPredictionHorizon,
        )
        self.globalPath: Optional[FloatArray] = None

    def setGlobalPath(self, globalPath: FloatArray) -> None:
        """Sets Global Path."""
        self.globalPath = globalPath

    def calculatePathInGlobalFrame(
        self,
        cones: List[FloatArray],
        vehiclePosition: FloatArray,
        vehicleDirection: np.float_,
    ) -> FloatArray:
        """
        Calculates a path for the vehicle in the global frame based on the provided cones,
        vehicle position, and direction.

        Args:
            cones (List[FloatArray]): A list of NumPy arrays representing cone positions.
            vehiclePosition (FloatArray): The vehicle's current position as a NumPy array.
            vehicleDirection (np.float_): The vehicle's current heading direction in radians.

        Returns:
            FloatArray: The calculated path as a NumPy array.
        """

        if 0 < (len(cones[0]) + len(cones[1]) + len(cones[2])) < 3:  # blue, unknown, yellow
            cornerCasesPath = CornerCasesPath(vehiclePosition, vehicleDirection, cones)
            result = cornerCasesPath.getPath()
            self.calculatePath.previousPaths = np.concatenate(
                (self.calculatePath.previousPaths[-15:], result), axis=0
            )
            return result

        ### Cones Sorting ###
        coneSortingInput = ConeSortingInput(cones, vehiclePosition, vehicleDirection)
        self.coneSorting.setNewInput(coneSortingInput)
        sortedCones = self.coneSorting.runConeSorting()  # sortedCones = sortedLeft, sortedRight

        matchedConesInput = [np.zeros((0, 2)) for _ in ConeTypes]
        matchedConesInput[ConeTypes.left] = sortedCones[0]
        matchedConesInput[ConeTypes.right] = sortedCones[1]

        if 0 < len(sortedCones[0]) < 3 and 0 < len(sortedCones[1]) < 3:
            cornerCasesPath = CornerCasesPath(vehiclePosition, vehicleDirection, matchedConesInput)
            result = cornerCasesPath.getPath()
            self.calculatePath.previousPaths = np.concatenate(
                (self.calculatePath.previousPaths[-15:], result), axis=0
            )
            return result

        ### Cone Matching ###
        coneMatchingInput = ConeMatchingInput(matchedConesInput, vehiclePosition, vehicleDirection)
        self.coneMatching.setNewInput(coneMatchingInput)
        (
            leftConesWithVirtual,
            rightConesWithVirtual,
            leftToRightMatch,
            rightToLeftMatch,
        ) = self.coneMatching.runConeMatching()

        ### Path Calculation ###
        pathCalculationInput = PathCalculationInput(
            leftConesWithVirtual,
            rightConesWithVirtual,
            leftToRightMatch,
            rightToLeftMatch,
            vehiclePosition,
            vehicleDirection,
            self.globalPath,
        )
        self.calculatePath.setNewInput(pathCalculationInput)

        return self.calculatePath.runPathCalculation()
