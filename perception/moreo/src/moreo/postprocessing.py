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
    def post_processing(self,my_array:npt.NDArray[np.float32],
        my_size:npt.NDArray[np.float32],
        top_left:npt.NDArray[np.float32])-> npt.NDArray[np.float32]:
        """
        This function is used to postprocess the output of the neural network.

            Parameters:
            my_array (numpy.ndarray): The output of the neural network.
            my_size (numpy.ndarray): The size of the image.
            top_left (numpy.ndarray): The top left corner of the image.

            Returns:
            numpy.ndarray: The postprocessed output of the neural network.
        """
        post_array=np.hstack((my_size,my_size,my_size,my_size,my_size,my_size,my_size,my_size))*my_array
        post_array+=np.hstack((top_left,top_left,top_left,top_left,top_left,top_left,top_left,top_left))
        return post_array
my_array1=np.random.randint(0,54,(610,16))
for_resizing=np.random.randint(0,54,(610,2))
top_left1=np.random.randint(0,54,(610,2))
SUM = 0
ITERS = 100000

for i in range(ITERS):
    a = datetime.datetime.now()
    postprocessor = Postprocessing() 
    postprocessed_array = postprocessor.post_processing(my_array1, for_resizing, top_left1) 
    b = datetime.datetime.now()
    SUM+=(b - a).microseconds*1e-3

print(SUM/ITERS, 'milliseconds')
