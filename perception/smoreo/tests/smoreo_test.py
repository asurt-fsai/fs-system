"""
Test file for smoreo
"""
import unittest
from smoreo.smoreo import Smoreo
from asurt_msgs.msg import LandmarkArray
import numpy as np


class SmoreoTest(unittest.TestCase):
    """
    Test class for smoreo
    """

    def setUp(self) -> None:
        self.correctParams = {
            "cut_off_y": 0.5,
            "camera_height_from_ground": 0.5,
            "cx": 0.5,
            "cy": 0.5,
            "f": 0.5,
            "worldCords_inCamera": np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]]),
            "cone_height": 0.5,
        }

    def testParamInp(self) -> None:
        """
        Test if the params are passed correctly
        """
        smoreo = Smoreo(self.correctParams)
        self.assertEqual(smoreo.params, self.correctParams)

    def testFrameId(self) -> None:
        """
        Test if the frame id is passed correctly
        """
        frame = "flir"
        smoreo = Smoreo(self.correctParams)
        self.assertEqual(smoreo.allLandMarks.header.frame_id, frame)

    def testRaisingErrorAtInvalidParam(self) -> None:
        """
        Test if the error is raised when invalid params are passed
        """
        with self.assertRaises(TypeError):
            Smoreo("params")

    def testTypeLandmarkArray(self) -> None:
        """
        Test if the type of landmark array is correct
        """
        smoreo = Smoreo(self.correctParams)
        self.assertIsInstance(smoreo.allLandMarks, LandmarkArray)

    def testTypeErrorWithNoneParam(self) -> None:
        """
        Test if the error is raised when params are None
        """
        with self.assertRaises(TypeError):
            Smoreo(None)

    def testInvalidParam(self) -> None:
        """
        Test if the error is raised when invalid params are passed
        """
        with self.assertRaises(TypeError):
            wrongParams = self.correctParams
            wrongParams["cut_off_y"] = "cy"
            Smoreo(wrongParams)
        # self.assertEqual(str(context.exception),"Invalid Param")

    def testRaisingErrorAtInvalidBboxcy(self) -> None:
        """
        Test if the error is raised when invalid bboxcy is passed
        """
        smoreo = Smoreo(self.correctParams)
        with self.assertRaises(TypeError):
            smoreo.filterNearBoxes("cy")

    def testRaisingErrorAtInvalidCutoffY(self) -> None:
        """
        Test if the type of cut off y is correct
        """
        with self.assertRaises(TypeError):
            Smoreo({"cut_off_y": "cy"})

    def testReturnTypeFilterNearBoxes(self) -> None:
        """
        Test if the return type of filterNearBoxes is correct
        """
        newParams = self.correctParams
        newParams["cut_off_y"] = 7.9
        smoreo = Smoreo(newParams)
        self.assertIsInstance(smoreo.filterNearBoxes(0.5), bool)

    def testOutputofInvalidBboxcy(self) -> None:
        """
        Test if the output of invalid bboxcy is correct
        """
        newParams = self.correctParams
        newParams["cut_off_y"] = 50.0
        smoreo = Smoreo(newParams)
        self.assertFalse(smoreo.filterNearBoxes(75.0))

    def testOutputofValidBboxcy(self) -> None:
        """
        Test if the output of valid bboxcy is correct
        """
        newParams = self.correctParams
        newParams["cut_off_y"] = 50.0
        smoreo = Smoreo(newParams)
        self.assertTrue(smoreo.filterNearBoxes(25.0))

    def testOutputofEqualBboxcy(self) -> None:
        """
        Test if the output of equal bboxcy to the cutoffy is valid
        """
        newParams = self.correctParams
        newParams["cut_off_y"] = 50.0
        smoreo = Smoreo(newParams)
        self.assertTrue(smoreo.filterNearBoxes(50.0))


if __name__ == "__main__":
    unittest.main()
