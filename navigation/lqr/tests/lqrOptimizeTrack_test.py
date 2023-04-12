#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import unittest
import numpy as np
from lqr import Track, SolverMatrices, setupMatrices, prepTrack, optimizeMinCurve

refTrack = np.zeros((4, 4))
refTrack[:, 0] = 5 * np.cos(np.linspace(0, 2 * np.pi, 4))
refTrack[:, 1] = 5 * np.sin(np.linspace(0, 2 * np.pi, 4))
refTrack[:, 2] = 0.5
refTrack[:, 3] = 0.5

trackClass = Track(refTrack=refTrack)
solverMatrices = SolverMatrices()
(
    trackClass.smooth.track,
    trackClass.smooth.normVectors,
    trackClass.smooth.alpha,
    trackClass.smooth.xCoeff,
    trackClass.smooth.yCoeff,
    trackClass.smooth.noPoints,
    trackClass.smooth.noSplines,
) = prepTrack(refTrack=trackClass.original)


class test_lqrOptimizeTrack(unittest.TestCase):
    def test_setupMatrices(self):
        setupMatrices(trackClass, solverMatrices)

    def test_optimizeMinCurve(self):
        optimizeMinCurve(trackClass, solverMatrices)


if __name__ == "__main__":
    unittest.main()
