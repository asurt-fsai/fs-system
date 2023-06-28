# pylint: disable=all
# mypy: ignore-errors
import unittest
from pure_pursuit import PidController


class test_pid_controller(unittest.TestCase):
    def test_proportionalControl(self):
        pidcontroller = PidController()
        self.assertAlmostEqual(pidcontroller.proportionalControl(10.0, 5.0, 1.0)[0], 4.65)

    def test_throttleCalc(self):
        pidcontroller = PidController()
        self.assertAlmostEqual(pidcontroller.throttleControl(3), 0.995, 3)


if __name__ == "__main__":

    unittest.main()
