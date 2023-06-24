#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import unittest

# from pure_pursuit import State, States, WayPoints, purepursuitSteercontrol, Position
from dataclasses import dataclass
from pure_pursuit import WayPoints, State, purepursuitSteercontrol, Position

# from pure_pursuit.pure_pursuit import State
import numpy as np
from nav_msgs.msg import Odometry


class test_purepursuit_controller(unittest.TestCase):
    def test_rearX(self):
        # firstPose = Position(0,0)
        pose = Odometry()
        firstpose = Position(0, 0)
        nextpose = Odometry()
        nextpose.pose.pose.position.x = 12
        nextpose.pose.pose.position.y = 12
        pose.pose.pose.position.x = 5
        pose.pose.pose.position.y = 5
        state = State(firstpose, 0, 0)
        state.update(pose)
        # print("old X:",state.oldPosition.x, "old Y:", state.oldPosition.y, "updated X:", state.position.x, "updated Y:", state.position.y, "current speed:", state.currentSpeed)
        state.update(nextpose)
        # print("old X:",state.oldPosition.x, "old Y:", state.oldPosition.y, "updated X:", state.position.x, "updated Y:", state.position.y, "current speed:", state.currentSpeed)
        self.assertAlmostEqual(state.rearX, 7.0, -1)
        self.assertAlmostEqual(state.rearY, 0.0, -1)

    def test_calcDistance(self):
        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0
        # self.assertAlmostEqual(state.calcDistance(3.0, 4.0), 7.0, 1)

    def test_waypoints(self):
        waypoints = WayPoints()
        waypoints.waypoints = []
        # self.assertIsNotNone(waypoints.waypoints[0])

    # def test_searchTargetIndex(self):
    #     states = WayPoints()
    #     states.xList = [1,2,3,4,5]
    # self.assertEqual(states.searchTargetIndex(3), 2)
    def test_purepursuitController(self):
        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0

        # self.assertAlmostEqual(state.purepursuitController(3.0, 4.0), 7.0, 1)

    def test_purepursuit(self):
        ind = 0
        state = State(Position(0, 0), 0, 0)
        state.currentSpeed = 0.0
        waypoints = WayPoints()
        waypoints.xList = [1, 20, 30, 40, 50]
        waypoints.yList = [19, 20, 33, 40, 50]
        # self.assertAlmostEqual(purepursuitSteercontrol(state,waypoints,2)[0], 200.0, 1)

    def test_steerControlMethod(self):
        waypoints = WayPoints()
        waypoints.xList = [1, 20, 30, 40, 50]
        waypoints.yList = [19, 20, 33, 40, 50]
        state = State(Position(0, 0), 0, 0)
        delta = purepursuitSteercontrol(state, waypoints, 0)
        print(delta)


if __name__ == "__main__":

    unittest.main()
