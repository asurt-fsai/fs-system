"""
.
"""
from nav_msgs.msg import Path
import rospy
import numpy.typing as npt
import numpy as np
import tf2_ros


class OriginalTrack:
    """
    This class contains the original track data.
    path: The path coordinate points + the width of the track [x,y,w_right,w_left]
    noPoints: The number of points in the track
    distCum: The cumulative distance of each point of the input track
    noSplines: The number of splines in the track
    xCoeff: The x coefficients for the splines
    yCoeff: The y coefficients for the splines
    normVectors: The normal vectors of the track at each point
    """

    def __init__(self, recievedPath: npt.NDArray[np.float64]):
        recievedPath = np.vstack((recievedPath, recievedPath[0]))
        self.path = recievedPath
        self.noPoints = self.path.shape[0]
        self.distCum = self.calcDistCum()
        self.noSplines: float
        self.alpha: npt.NDArray[np.float64]
        self.normVectors: npt.NDArray[np.float64]

    def calcDistCum(self) -> npt.NDArray[np.float64]:
        """
        Calculates the cumulative distance of each point of the input track

        Parameters
        ----------
        self

        Returns
        -------
        distsCumulative: np.array, shape=(N,1)
            Cumulative distances of the points.
        """
        distsCumulative: npt.NDArray[np.float64] = np.cumsum(
            np.sqrt(np.sum(np.power(np.diff(self.path[:, :2], axis=0), 2), axis=1))
        )
        distsCumulative = np.insert(distsCumulative, 0, 0.0)
        return distsCumulative

    def pathToNumpy(self, path: Path) -> npt.NDArray[np.float64]:
        """
        Converts a path message to a numpy array
        and transforms the coordinates to the world frame

        Parameters
        ----------
        path: Path
            Path message

        Returns
        -------
        pathArray: np.ndarray, shape=(n, 2)
            Numpy array with x and y coordinates of the path
        """
        tfBuffer = tf2_ros.Buffer()
        pathArray = np.zeros((len(path.poses), 2))
        for index, pose in enumerate(path.poses):
            globalPose = tfBuffer.transform(pose, "world", rospy.Duration(1))
            pathArray[index, 0] = globalPose.pose.position.x
            pathArray[index, 1] = globalPose.pose.position.y
        return pathArray

    # Original
