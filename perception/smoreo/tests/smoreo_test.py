"""
Test file for smoreo
"""
import unittest
import numpy as np
from smoreo.smoreo import Smoreo
from asurt_msgs.msg import LandmarkArray

class SmoreoTest(unittest.TestCase):
    """
    Test class for smoreo
    """
    def testParamInp(self):
        """
        Test if the params are passed correctly
        """
        dictParam={"cut_off_y": 0.5}
        smoreo = Smoreo(dictParam)
        self.assertEqual(smoreo.params, dictParam)
    def testFrameId(self):
        """
        Test if the frame id is passed correctly
        """
        frame="flir"
        smoreo = Smoreo({})
        self.assertEqual(smoreo.allLandMarks.header.frame_id, frame)    
    def testInvalidTypeParam(self):
        """
        Test if the error is raised when invalid params are passed
        """
        with self.assertRaises(TypeError):
            Smoreo("params")
    def testTypeLandmarkArray(self):
        """
        Test if the type of landmark array is correct
        """
        smoreo = Smoreo({})
        self.assertIsInstance(smoreo.allLandMarks, LandmarkArray)
    def testTypeErrorWithNoneParam(self):
        """
        Test if the error is raised when params are None
        """
        with self.assertRaises(TypeError):
            Smoreo(None)
    def testInvalidParam(self):
        """
        Test if the error is raised when invalid params are passed
        """
        with self.assertRaises(ValueError):
            Smoreo({"cut_off_y":"cy"})
        #self.assertEqual(str(context.exception),"Invalid Param") 
    def testInvalidBboxcy(self):
        """
        Test if the error is raised when invalid bboxcy is passed
        """
        smoreo = Smoreo({"cut_off_y":7.9})
        with self.assertRaises(TypeError):
            smoreo.filterNearBoxes("cy")
    def testInvalidCutoffY(self):
        """
        Test if the type of cut off y is correct
        """
        with self.assertRaises(TypeError):
            Smoreo({"cut_off_y":"cy"})
    def testReturnTypeFilterNearBoxes(self):
        """
        Test if the return type of filterNearBoxes is correct
        """
        smoreo = Smoreo({"cut_off_y":7.9})
        self.assertIsInstance(smoreo.filterNearBoxes(0.5), bool)
    def testOutputofInvalidBboxcy(self):
        """
        Test if the output of invalid bboxcy is correct
        """
        smoreo = Smoreo({"cut_off_y":50})
        self.assertFalse(smoreo.filterNearBoxes(75))
    def testOutputofValidBboxcy(self):
        """
        Test if the output of valid bboxcy is correct
        """
        smoreo = Smoreo({"cut_off_y":50})
        self.assertTrue(smoreo.filterNearBoxes(25))
    def testOutputofEqualBboxcy(self):
        """
        Test if the output of equal bboxcy to the cutoffy is valid
        """
        smoreo = Smoreo({"cut_off_y":50})
        self.assertTrue(smoreo.filterNearBoxes(50))
    def testInvalidPose(self):
        """
        Test if the error is raised when pose is not a numpy array of size (1,3)
        """
        smoreo = Smoreo({})
        box=np.array([1,2,3,4,5,6],dtype=np.float32)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray([1,2,3],box)
        with self.assertRaises(IndexError):
            smoreo.addToLandmarkArray("pose", box)
        with self.assertRaises(IndexError):
            smoreo.addToLandmarkArray(np.zeros((2,2),dtype=np.float32),box)
    def testInvalidBox(self):
        """
        Test if the error is raised when box is not a numpy array of size (1,6)
        """
        smoreo = Smoreo({})
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(np.zeros((1,3),dtype=np.float32),99)
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(np.zeros((1,3),dtype=np.int64),99)
        with self.assertRaises(KeyError):
            smoreo.addToLandmarkArray(np.zeros((1,3),dtype=np.float32),{})
        pose=np.array([1,2,3],dtype=np.float32)
        with self.assertRaises(IndexError):
            smoreo.addToLandmarkArray(pose, "box")
        with self.assertRaises(IndexError):
            smoreo.addToLandmarkArray(pose,np.zeros((2,2),dtype=np.float32))  
        with self.assertRaises(TypeError):
            smoreo.addToLandmarkArray(np.zeros((1,3),dtype=np.float32),[1,2,3,4,5,6]) 
    def testAddToLandmarkArrayArgs(self):
        """
        Test if the error is raised when the arguments are not passed in the correct order
        """
        smoreo=Smoreo({})
        pose=np.array([1,2,3],dtype=np.float32)
        box=np.array([1,2,3,4,5,6],dtype=np.float32)
        with self.assertRaises(IndexError):
            smoreo.addToLandmarkArray(box, pose)
    def testPoseOrder(self):
        """
        Test if the pose is added in the correct order
        """
        smoreo=Smoreo({})
        pose1 = np.array([[1, 2, 3]], dtype=np.float32)
        pose2 = np.array([[3, 1, 2]], dtype=np.float32)
        box=np.array([1,2,3,4,5,6],dtype=np.float32)
        smoreo.addToLandmarkArray(pose1, box)
        with self.assertRaises(IndexError):
            smoreo.addToLandmarkArray(pose2, box)
    def testBoxOrder(self):
        """
        Test if the box is added in the correct order
        """
        smoreo=Smoreo({})
        pose=np.zeros((1,3),dtype=np.float32)
        box1=np.array([1,2,3,4,5,6],dtype=np.float32)
        box2=np.array([1,2,3,4,5,6],dtype=np.float32)
        smoreo.addToLandmarkArray(pose, box1)
        with self.assertRaises(IndexError):
            smoreo.addToLandmarkArray(pose, box2)
    def testAddToLandmarkArray(self):
        """
        Test if the landmark array is updated correctly
        """
        smoreo=Smoreo({})
        pose=np.zeros((1,3),dtype=np.float32)
        box=np.array([1,2,3,4,5,6],dtype=np.float32)
        smoreo.addToLandmarkArray(pose, box)
        self.assertEqual(len(smoreo.allLandMarks.landmarks), 1)
        self.assertEqual(smoreo.allLandMarks.landmarks[0].position.x, pose[0][0])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].position.y, pose[0][1])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].position.z, pose[0][2])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].identifier, box[4])
        self.assertEqual(smoreo.allLandMarks.landmarks[0].type, box[5])
        pose2 = np.array([[4.0, 5.0, 6.0]], dtype=np.float32)
        box2 = np.array([7.0, 8.0, 9.0, 10.0, 11.0, 12.0], dtype=np.float32)
        smoreo.addToLandmarkArray(pose2, box2)
        self.assertEqual(len(smoreo.allLandMarks.landmarks), 2)
if __name__=='__main__':
    unittest.main()
