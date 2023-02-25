"""
Test file for smoreo
"""
import unittest
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
    def testRaisingErrorAtInvalidParam(self):
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
    def testRaisingErrorAtInvalidBboxcy(self):
        """
        Test if the error is raised when invalid bboxcy is passed
        """
        smoreo = Smoreo({"cut_off_y":7.9})
        with self.assertRaises(TypeError):
            smoreo.filterNearBoxes("cy")
    def testRaisingErrorAtInvalidCutoffY(self):
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
if __name__=='__main__':
    unittest.main()
    