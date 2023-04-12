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
    def test_numpyToPath(self):
        path = np.array([[1, 2], [3, 4]])
        pathMessage = numpyToPath(path)
        self.assertIsInstance(pathMessage, Path)
        self.assertIsInstance(pathMessage.poses[0], PoseStamped)
        self.assertEqual(pathMessage.poses[0].pose.position.x, 1)
        self.assertEqual(pathMessage.poses[0].pose.position.y, 2)
        self.assertEqual(pathMessage.poses[1].pose.position.x, 3)
        self.assertEqual(pathMessage.poses[1].pose.position.y, 4)

    def test_normVecsToTrackBound(self):
        track = Track(np.zeros(1))
        track.smooth.track = np.array(
            [0, 0, 0.5, 0.5], [1, 0, 0.5, 0.5], [2, 0, 0.5, 0.5], [3, 0, 0.5, 0.5]
        )
        track.smooth.normVectors = np.array([0, 1], [0, 1], [0, 1], [0, 1])
        trackUpBound, trackLowBound = normVecsToTrackBound(track)
        self.assertAlmostEqual(trackUpBound, np.array([0, 0.5], [1, 0.5], [2, 0.5], [3, 0.5]))
        self.assertAlmostEqual(trackLowBound, np.array([0, -0.5], [1, -0.5], [2, -0.5], [3, -0.5]))

    def test_interpSplines(self):
        track = Track(np.zeros(1))
        track.smooth.track = np.array([0, 0, 0.5, 0.5], [2, 0, 0.5, 0.5])
        track.smooth.normVectors = np.array([0, 1], [0, 1])
        track.smooth.xCoeff = np.array([0, 0, 0, 1], [0, 0, 0, 1])
        track.smooth.yCoeff = np.array([0, 0, 0, 1], [0, 0, 0, 1])
        interpRaceline = interpSplines(track)
        self.assertAlmostEqual(interpRaceline, np.array([0, 0], [1, 0], [2, 0]))

    def test_calcSplineLengths(self):
        track = Track(np.zeros(1))
        track.smooth.track = np.array([0, 0, 0.5, 0.5], [2, 0, 0.5, 0.5])
        track.smooth.normVectors = np.array([0, 1], [0, 1])
        track.smooth.xCoeff = np.array([0, 0, 0, 1], [0, 0, 0, 1])
        track.smooth.yCoeff = np.array([0, 0, 0, 1], [0, 0, 0, 1])
        splineLengths = calcSplineLengths(track)
        self.assertAlmostEqual(splineLengths, np.array([1, 1]))


if __name__ == "__main__":
    unittest.main()
