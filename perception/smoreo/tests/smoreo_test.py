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
    def testTypeParam(self):
        """
        Test if the type of params is correct
        """
        smoreo = Smoreo({})
        self.assertIsInstance(smoreo.params, dict)
    def testTypeLandmarkArray(self):
        """
        Test if the type of landmark array is correct
        """
        smoreo = Smoreo({})
        self.assertIsInstance(smoreo.allLandMarks, LandmarkArray) 
    def testTypeCutoffY(self):
        """
        Test if the type of cut off y is correct
        """
        smoreo = Smoreo({"cut_off_y":7.9})  
        self.assertIsInstance(smoreo.params["cut_off_y"], float)
    def testReturnTypeFilterNearBoxes(self):
        """
        Test if the return type of filterNearBoxes is correct
        """
        smoreo = Smoreo({"cut_off_y":7.9})
        self.assertIsInstance(smoreo.filterNearBoxes(0.5), bool)
if __name__=='__main__':
    unittest.main()
    