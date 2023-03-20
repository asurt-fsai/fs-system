"""
Test file for smoreo
"""
import os
import unittest
import pickle
import numpy as np
from asurt_msgs.msg import LandmarkArray
from tf_helper.utils import parseLandmarks
from smoreo.smoreo import Smoreo


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

    def testInvalidBboxcy(self) -> None:
        """
        Test if the error is raised when invalid bboxcy is passed
        """
        with self.assertRaises(TypeError):
            wrongParams = self.correctParams
            wrongParams["cut_off_y"] = "cy"
            Smoreo(wrongParams)

    def testRaisingErrorAtInvalidBboxcy(self) -> None:
        """
        Test if the error is raised when invalid bboxcy is passed
        """
        smoreo = Smoreo(self.correctParams)
        with self.assertRaises(TypeError):
            smoreo.filterNearBoxes("cy")

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

    def testInvalidPose(self) -> None:
        """
        Test if the error is raised when pose is not a numpy array of size (1,3)
        """
        smoreo = Smoreo(self.correctParams)
        box = np.array([1, 2, 3, 4, 5, 6, 7], dtype=np.float32)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray([1, 2, 3], box)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray("pose", box)
        with self.assertRaises(ValueError):
            smoreo.addToLandmarkArray(np.zeros((2, 2), dtype=np.float32), box)

    def testInvalidBox(self) -> None:
        """
        Test if the error is raised when box is not a numpy array of size (1,6)
        """
        smoreo = Smoreo(self.correctParams)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(np.zeros((1, 3), dtype=np.float32), 99)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(np.zeros((1, 3), dtype=np.int64), 99)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(np.zeros((1, 3), dtype=np.float32), {})
        pose = np.array([1, 2, 3], dtype=np.float32)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(pose, "box")
        with self.assertRaises(ValueError):
            smoreo.addToLandmarkArray(pose, np.zeros((2, 2), dtype=np.float32))
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(np.zeros((1, 3), dtype=np.float32), [1, 2, 3, 4, 5, 6, 7])

    def testAddToLandmarkArray(self) -> None:
        """
        Test if the landmark array is updated correctly
        """
        smoreo = Smoreo(self.correctParams)
        pose = np.zeros((1, 3), dtype=np.float32)
        box = np.array([1, 2, 3, 4, 5, 6, 7], dtype=np.float32)
        smoreo.addToLandmarkArray(pose, box)
        self.assertEqual(len(smoreo.allLandMarks.landmarks), 1)
        self.assertEqual(smoreo.allLandMarks.landmarks[0].position.x, pose[0][0])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].position.y, pose[0][1])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].position.z, pose[0][2])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].identifier, box[4])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].type, box[5])
        pose2 = np.array([[4.0, 5.0, 6.0]], dtype=np.float32)
        box2 = np.array([7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0], dtype=np.float32)
        smoreo.addToLandmarkArray(pose2, box2)
        self.assertEqual(len(smoreo.allLandMarks.landmarks), 2)

    def testPredictWithConeBase(self) -> None:
        """
        Test if the prediction is correct with cone base
        """
        testCasePath = r"../testing/testCase1.pickle"
        scriptDir = os.path.dirname(__file__)
        absFilePath = os.path.join(scriptDir, testCasePath)
        with open(
            absFilePath,
            "rb",
        ) as file:
            testCase = pickle.load(file)

        smoreo = Smoreo(testCase["params"])
        landmarks = smoreo.predictWithBase(testCase["bboxes"])
        cones = parseLandmarks(landmarks)
        np.testing.assert_array_equal(cones, testCase["predictedCones"])


if __name__ == "__main__":
    unittest.main()
