# pylint: disable=all
# mypy: ignore-errors
import unittest
from pure_pursuit import PidController, State, WayPoints, Position


class test_pid_controller(unittest.TestCase):
    def test_proportionalControl(self):
        pidcontroller = PidController()
        self.assertAlmostEqual(pidcontroller.proportionalControl(102.0, 5.0, 1.2)[0], -1)

    def test_distanceCalc(self):

        state = State(Position(0, 0), 0, 0)
        state.rearX = 0.0
        state.rearY = 0.0
        self.assertAlmostEqual(state.calcDistance(3.0, 4.0), 7.0, -1)

    def test_throttleCalc(self):
        pidcontroller = PidController()
        self.assertAlmostEqual(PidController.throttleControl(pidcontroller, 122), -0.8, 1)

    def test_purepursuitControl(self):

        waypoint = WayPoints()
        waypoint.xList = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        waypoint.yList = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
        state = State
        state.x = 1
        state.y = 2
        state.rearX = 5
        state.rearY = 4
        state.currentSpeed = 1
        # waypoint.searchTargetIndex(state)
        self.assertAlmostEqual(waypoint.searchTargetIndex(state)[1], 1)


if __name__ == "__main__":

    unittest.main()
