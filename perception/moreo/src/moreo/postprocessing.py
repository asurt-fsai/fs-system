"""
This file is used to postprocess the output of the neural network.
"""
import datetime
import numpy as np
import numpy.typing as npt

class Postprocessing:
    """
    This class is used to postprocess the output of the neural network.
    """
    def postProcessing(self,myArray:npt.NDArray[np.float32],
        mySize:npt.NDArray[np.float32],
        topLeft:npt.NDArray[np.float32])-> npt.NDArray[np.float32]:
        """
        This function is used to postprocess the output of the neural network.

            Parameters:
            myArray (numpy.ndarray): The output of the neural network.
            mySize (numpy.ndarray): The size of the image.
            topleft (numpy.ndarray): The top left corner of the image.

            Returns:
            numpy.ndarray: The postprocessed output of the neural network.
        """
        postArray=np.hstack((mySize,mySize,mySize,mySize,mySize,mySize,mySize,mySize))*myArray
        postArray+=np.hstack((topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft,topLeft))
        return postArray
my_array1=np.random.randint(0,54,(610,16))
for_resizing=np.random.randint(0,54,(610,2))
top_left1=np.random.randint(0,54,(610,2))
SUM = 0
ITERS = 100000

for i in range(ITERS):
    a = datetime.datetime.now()
    postprocessor = Postprocessing() 
    postprocessed_array = postprocessor.postProcessing(my_array1, for_resizing, top_left1) 
    b = datetime.datetime.now()
    SUM+=(b - a).microseconds*1e-3

print(SUM/ITERS, 'milliseconds')
