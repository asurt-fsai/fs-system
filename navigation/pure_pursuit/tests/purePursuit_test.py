#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
import unittest
from dataclasses import dataclass
from pure_pursuit import WayPoints, State, purepursuitSteercontrol, Position


import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


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

        self.assertAlmostEqual(state.rearX, 10.75)

    def test_calcDistance(self):
        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0
        self.assertAlmostEqual(state.calcDistance(3.0, 4.0), 5.0)

    def test_purepursuitController(self):
        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0
        waypoints = WayPoints()
        pathSend = Path()
        pose1 = PoseStamped()
        pose2 = PoseStamped()
        pose3 = PoseStamped()
        pose4 = PoseStamped()
        pose5 = PoseStamped()
        pose1.pose.position.x = 0.1
        pose1.pose.position.y = 0.1
        pose2.pose.position.x = 0.4
        pose2.pose.position.y = 0.5
        pose3.pose.position.x = 0.6
        pose3.pose.position.y = 0.8
        pose4.pose.position.x = 40
        pose4.pose.position.y = 40
        pose5.pose.position.x = 50
        pose5.pose.position.y = 50

        pathSend.poses = [pose1, pose2, pose3, pose4, pose5]
        waypoints.waypoints = pathSend
        waypoints.points = pathSend.poses

        _, ind = purepursuitSteercontrol(state, waypoints, 5)
        self.assertAlmostEqual(ind, 4)

    def test_searchTargetIndexMethod(self):

        state = State(Position(0, 0), 0, 0)
        state.currentSpeed = 0.0
        waypoints = WayPoints()
        pathSend = Path()
        pose1 = PoseStamped()
        pose2 = PoseStamped()
        pose3 = PoseStamped()
        pose4 = PoseStamped()
        pose5 = PoseStamped()
        pose1.pose.position.x = 0.1
        pose1.pose.position.y = 0.1
        pose2.pose.position.x = 0.4
        pose2.pose.position.y = 0.5
        pose3.pose.position.x = 0.6
        pose3.pose.position.y = 0.8
        pose4.pose.position.x = 40
        pose4.pose.position.y = 40
        pose5.pose.position.x = 50
        pose5.pose.position.y = 50
        pathSend.poses = [pose1, pose2, pose3, pose4, pose5]
        waypoints.waypoints = pathSend
        waypoints.points = pathSend.poses
        ind, _ = waypoints.searchTargetIndex(state)
        self.assertAlmostEqual(ind, 3)

    def test_steerControlMethod(self):
        waypoints = WayPoints()
        pathSend = Path()
        pose1 = PoseStamped()
        pose2 = PoseStamped()
        pose3 = PoseStamped()
        pose1.pose.position.x = 0.0
        pose1.pose.position.y = 0.0
        pose2.pose.position.x = 6.1
        pose2.pose.position.y = 6.5
        pose3.pose.position.x = 30
        pose3.pose.position.y = 33
        pathSend.poses = [pose1, pose2, pose3]
        waypoints.waypoints = pathSend
        waypoints.points = pathSend.poses
        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0
        delta, _ = purepursuitSteercontrol(state, waypoints, 0)

        self.assertAlmostEqual(delta, 0.97, places=2)


if __name__ == "__main__":

    unittest.main()
