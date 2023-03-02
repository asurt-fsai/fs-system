"""
This file is used to test the postprocessing class.
"""
import unittest
import numpy as np
from moreo.postprocessing import Postprocessing

class TestPostProcessing(unittest.TestCase):
    """
    This class is used to test the postprocessing class.
    """
    def test_functionality(self)-> None:
        """
        This function is used to test the functionality of the postprocessing class.
        """
        my_array1=np.random.randint(0,54,(610,16))
        for_resizing=np.random.randint(0,54,(610,2))
        top_left1=np.random.randint(0,54,(610,2))
        postprocessor = Postprocessing() 
        postprocessed_array = postprocessor.postProcessing(my_array1, for_resizing, top_left1) 
        self.assertEqual(postprocessed_array.shape, (610, 16))
        for i in range(610):
            for j in range(16):
                self.assertEqual(postprocessed_array[i][j],
                my_array1[i][j]*for_resizing[i][0]+top_left1[i][0])
                self.assertEqual(postprocessed_array[i][j+1],
                my_array1[i][j+1]*for_resizing[i][1]+top_left1[i][1])
                j+=2
if __name__ == "__main__":
    unittest.main()
