#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import unittest
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from lqr import numpyToPath, normVecsToTrackBound, Track, interpSplines, calcSplineLengths


class test_lqrPrepareTrack(unittest.TestCase):
    def test_normVecsToTrackBound(self):
        track = Track(np.zeros(1))
        track.smooth.track = np.array(
            [[0, 0, 0.5, 0.5], [1, 0, 0.5, 0.5], [2, 0, 0.5, 0.5], [3, 0, 0.5, 0.5]]
        )
        track.smooth.normVectors = np.array([[0, 1], [0, 1], [0, 1], [0, 1]])
        trackUpBound, trackLowBound = normVecsToTrackBound(track)
        self.assertTrue(
            np.allclose(trackUpBound, np.array([[0, 0.5], [1, 0.5], [2, 0.5], [3, 0.5], [0, 0.5]]))
        )
        self.assertTrue(
            np.allclose(
                trackLowBound, np.array([[0, -0.5], [1, -0.5], [2, -0.5], [3, -0.5], [0, -0.5]])
            )
        )

    def test_interpSplines(self):
        xCoeffs = np.array([[0, 1, 0, 0], [0, 1, 0, 0]])
        yCoeffs = np.array([[0, 0, 0, 0], [0, 0, 0, 0]])
        splineLengths = np.array([1, 1])
        stepSize = 0.5
        interpRaceline = interpSplines(xCoeffs, yCoeffs, splineLengths, stepSize)
        self.assertTrue(
            np.allclose(interpRaceline, np.array([[0, 0], [0.5, 0], [0, 0], [0.5, 0], [1, 0]]))
        )

    def test_calcSplineLengths(self):
        xCoeffs = np.array([[0, 1, 0, 0], [0, 1, 0, 0]])
        yCoeffs = np.array([[0, 0, 0, 0], [0, 0, 0, 0]])
        splineLengths = calcSplineLengths(xCoeffs, yCoeffs)
        self.assertTrue(np.allclose(splineLengths, np.array([1, 1])))


if __name__ == "__main__":
    unittest.main()
