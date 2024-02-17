#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Description: A class that runs the whole path planning pipeline.

- Cone sorting
- Cone Matching
- Path Calculation

"""
from __future__ import annotations

from typing import Any, List, Optional, Union

import numpy as np
from numpy.typing import NDArray

from cones_sorting.cone_sorting_wrapper import ConeSortingInput, ConeSorting
from cones_sorting.core_cone_sorter import ConeSorter
from cone_matching.core_cone_matching import ConeMatching, ConeMatchingInput
from calculate_path.core_calculate_path import CalculatePath, PathCalculationInput

from utils.cone_types import ConeTypes

class PathPlanner:
    def __init__(self):
        self.coneSorting = ConeSorting(
            maxNNeighbors=5,
            maxDist=6.5,
            maxDistToFirst=6.0,
            maxLength=12, #default=12
            thresholdDirectionalAngle=np.deg2rad(40),
            thresholdAbsoluteAngle=np.deg2rad(65),
            useUnknownCones=True,
        )
        self.coneMatching  = ConeMatching(
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

    def setGlobalPath(self, globalPath):
        self.globalPath = globalPath

    def calculatePathInGlobalFrame(
            self,
            cones: List[NDArray[np.float_]],
            vehiclePosition: NDArray[np.float_],
            vehicleDirection: np.float_,
    ) -> NDArray[np.float_]:
        
        ### Cones Sorting ###
        coneSortingInput = ConeSortingInput(
            cones, vehiclePosition, vehicleDirection
        ) 
        self.coneSorting.setNewInput(coneSortingInput)
        sortedLeft, sortedRight = self.coneSorting.runConeSorting()


        matchedConesInput = [np.zeros((0, 2)) for _ in ConeTypes] 
        matchedConesInput[ConeTypes.LEFT] = sortedLeft
        matchedConesInput[ConeTypes.RIGHT] = sortedRight
        ### Cone Matching ###
        coneMatchingInput = ConeMatchingInput(
            matchedConesInput,vehiclePosition,vehicleDirection
        )
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
