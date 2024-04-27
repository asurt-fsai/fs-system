import numpy as np

# import types for numpy arrays
from typing import List


class FastSLAM:
    def __init__(self, initPose=np.zeros((3, 1))):
        self.pose = initPose
        self.poseEstimate = initPose

    def estimatePose(self, odom: np.ndarray) -> np.ndarray:
        self.poseEstimate = self.poseEstimate + odom.reshape(3, 1)
        return self.poseEstimate
