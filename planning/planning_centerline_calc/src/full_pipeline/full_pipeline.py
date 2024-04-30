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

import numpy as np
from numpy.typing import NDArray

from src.cones_sorting.cone_sorting_wrapper import ConeSortingInput, ConeSorting
from src.cone_matching.core_cone_matching import ConeMatching, ConeMatchingInput
from src.calculate_path.core_calculate_path import CalculatePath, PathCalculationInput
from src.corner_case.corner_case_path import CornerCasesPath

from src.utils.cone_types import ConeTypes


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
    def __init__(self) -> None:
        self.coneSorting = ConeSorting(
            maxNNeighbors=5,
            maxDist=7,     #default=6.5
            maxDistToFirst=10.0,
            maxLength=7,  # default=12
            thresholdDirectionalAngle=np.deg2rad(40),
            thresholdAbsoluteAngle=np.deg2rad(65),
            useUnknownCones=True,
        )
        self.coneMatching = ConeMatching(
            minTrackWidth=3,
            maxSearchRange=5,
            maxSearchAngle=np.deg2rad(50),
            matchesShouldBeMonotonic=True,
        )
        self.calculatePath = CalculatePath(
            maximalDistanceForValidPath=5,
            mpcPathLength=20,
            mpcPredictionHorizon=40,
        )
        self.globalPath: Optional[NDArray[np.float_]] = None

    def setGlobalPath(self, globalPath: NDArray[np.float_]) -> None:
        """Sets Global Path."""
        self.globalPath = globalPath

    def calculatePathInGlobalFrame(
        self,
        cones: List[NDArray[np.float_]],
        vehiclePosition: NDArray[np.float_],
        vehicleDirection: np.float_,
    ) -> NDArray[np.float_]:
        """
        Calculates a path for the vehicle in the global frame based on the provided cones, 
        vehicle position, and direction.

        Args:
            cones (List[NDArray[np.float_]]): A list of NumPy arrays representing cone positions.
            vehiclePosition (NDArray[np.float_]): The vehicle's current position as a NumPy array.
            vehicleDirection (np.float_): The vehicle's current heading direction in radians.

        Returns:
            NDArray[np.float_]: The calculated path as a NumPy array.
        """
        
        ### Cones Sorting ###
        coneSortingInput = ConeSortingInput(cones, vehiclePosition, vehicleDirection)
        self.coneSorting.setNewInput(coneSortingInput)
        sortedLeft, sortedRight = self.coneSorting.runConeSorting()

        matchedConesInput = [np.zeros((0, 2)) for _ in ConeTypes]
        matchedConesInput[ConeTypes.left] = sortedLeft
        matchedConesInput[ConeTypes.right] = sortedRight
        
        if (len(sortedLeft) < 3 and len(sortedRight) < 3) and (len(sortedLeft) > 0 and len(sortedRight) > 0):
            cornerCasesPath = CornerCasesPath(vehiclePosition, vehicleDirection, matchedConesInput)
            return cornerCasesPath.getPath()
        
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
        result = self.calculatePath.runPathCalculation()

        return result
