#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import unittest
from dataclasses import dataclass
from pure_pursuit import WayPoints, State, purepursuitSteercontrol, Position


import numpy as np
from nav_msgs.msg import Odometry


class test_purepursuit_controller(unittest.TestCase):
    def test_rearX(self):

        pose = Odometry()
        firstpose = Position(0, 0)
        nextpose = Odometry()
        nextpose.pose.pose.position.x = 12
        nextpose.pose.pose.position.y = 12
        pose.pose.pose.position.x = 5
        pose.pose.pose.position.y = 5
        state = State(firstpose, 0, 0)
        state.update(pose)

        state.update(nextpose)

        self.assertAlmostEqual(state.rearX, 7.0, -1)

    def test_calcDistance(self):
        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0
        self.assertAlmostEqual(state.calcDistance(3.0, 4.0), 7.0, 1)

    def test_purepursuitController(self):
        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0
        waypoints = WayPoints()
        waypoints.xList = [1, 20, 30, 40, 50]
        waypoints.yList = [19, 20, 33, 40, 50]
        self.assertAlmostEqual(purepursuitSteercontrol(state, waypoints, 1)[0], 7.0)

    def test_purepursuit(self):

        state = State(Position(0, 0), 0, 0)
        state.currentSpeed = 0.0
        waypoints = WayPoints()
        waypoints.xList = [1, 20, 30, 40, 50]
        waypoints.yList = [19, 20, 33, 40, 50]
        self.assertAlmostEqual(purepursuitSteercontrol(state, waypoints, 2)[0], 200.0, 1)

    def test_steerControlMethod(self):
        waypoints = WayPoints()
        waypoints.xList = [1, 20, 30, 40, 50]
        waypoints.yList = [19, 20, 33, 40, 50]
        state = State(Position(0, 0), 0, 0)
        delta = purepursuitSteercontrol(state, waypoints, 0)

        self.assertAlmostEqual(delta, 0.0)


if __name__ == "__main__":

    unittest.main()
