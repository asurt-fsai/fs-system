"""
This module contains the reading objects
The base reading represents an empty reading.
"""

from typing import Dict, List, Any, Tuple
import numpy as np
import numpy.typing as npt
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from cv2 import KeyPoint  # pylint: disable=no-name-in-module


class BaseReading:
    """
    A base class for reading data from an image, odometry, and bounding boxes.
    """

    def __init__(self) -> None:
        """
        Initialize the base reading object.
        """

    def getImage(self) -> npt.NDArray[np.float64]:
        """
        Get the image data.
        :return: An ndarray representation of the image data
        """
        raise ValueError("Trying to access an empty reading")

    def getPosition(self) -> npt.NDArray[np.float64]:
        """
        Get the position of the reading.
        :return: A 1-D numpy array of x,y,z position of the reading
        """
        raise ValueError("Trying to access an empty reading")

    def getOrientation(self) -> npt.NDArray[np.float64]:
        """
        Get the orientation of the reading.
        :return: A 1-D numpy array of x,y,z,w quaternion
        representation of the orientation of the reading
        """
        raise ValueError("Trying to access an empty reading")

    def getBboxes(self) -> npt.NDArray[np.float64]:
        """
        Get the bounding boxes of the reading.
        :return: A numpy array of bounding boxes
        """
        raise ValueError("Trying to access an empty reading")

    def setFeaturesPerBbox(
        self,
        features: Dict[str, Tuple[npt.NDArray[np.float64], List[KeyPoint], npt.NDArray[np.int16]]],
    ) -> None:
        """
        Set features for each bounding box in the reading.

        :param features: A dictionary where the keys are strings
        representing the bounding box IDs and the values are
        tuples containing the bounding box, keypoints,
        and descriptors for each bounding box.
        :return: None
        """
        raise ValueError("Trying to access an empty reading")

    def getFeaturesPerBbox(
        self,
    ) -> Dict[str, Tuple[npt.NDArray[np.float64], List[KeyPoint], npt.NDArray[np.int16]]]:
        """
        Get the features for each bounding box in the reading.

        :return: A dictionary where the keys are strings representing
        the bounding box IDs and the values are tuples containing
        the bounding box, keypoints, and descriptors for each bounding box.
        """
        raise ValueError("Trying to access an empty reading")

    def getImageHeader(self) -> None:
        """
        Get the header of the image

        :return Header object of the image captured in this reading
        """
        raise ValueError("Trying to access an empty reading")


class Reading(BaseReading):
    """
    A concrete implementation of the BaseReading class
    that stores the image data, odometry, and bounding boxes.
    """

    def __init__(self, imgMsg: Image, odom: Odometry, bboxes: npt.NDArray[np.float64]) -> None:
        """
        Initialize the reading object with image message, odometry, and bounding boxes data.

        :param imgMsg: The image message of the reading.
        :param odom: The odometry of the image of the reading.
        :param bboxes: The bounding boxes associated with the image of the reading.
        """
        pose = odom.pose.pose
        pos = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        self.imgHeader = imgMsg.header
        self.image = np.asarray(
            np.frombuffer(imgMsg.data, dtype=np.uint8).reshape(imgMsg.height, imgMsg.width, -1)
        )
        self.position = np.asarray(pos[:3].reshape(-1, 1))
        self.orientation = np.asarray(pos[3:])
        self.bboxes = bboxes
        self.featuresPerBbox: Dict[str, Tuple[Any, Any, Any]] = {}

    def getImage(self) -> npt.NDArray[np.float64]:
        """
        Get the image data.
        :return: An ndarray representation of the image data
        """
        return self.image

    def getPosition(self) -> npt.NDArray[np.float64]:
        """
        Get the position of the reading.
        :return: A 1-D numpy array of x,y,z position of the reading
        """
        return self.position

    def getOrientation(self) -> npt.NDArray[np.float64]:
        """
        Get the orientation of the reading.
        :return: A 1-D numpy array of x,y,z,w quaternion
        representation of the orientation of the reading
        """
        return self.orientation

    def getBboxes(self) -> npt.NDArray[np.float64]:
        """
        Get the bounding boxes of the reading.
        :return: A numpy array of bounding boxes
        """
        return self.bboxes

    def setFeaturesPerBbox(
        self,
        features: Dict[str, Tuple[npt.NDArray[np.float64], List[KeyPoint], npt.NDArray[np.int16]]],
    ) -> None:
        """
        Set features for each bounding box in the reading.

        :param features: A dictionary where the keys are strings
        representing the bounding box IDs and the values are
        tuples containing the bounding box, keypoints, and descriptors for each bounding box.
        :return: None
        """
        self.featuresPerBbox = features

    def getFeaturesPerBbox(
        self,
    ) -> Dict[str, Tuple[npt.NDArray[np.float64], List[KeyPoint], npt.NDArray[np.int16]]]:
        """
        Get the features for each bounding box in the reading.

        :return: A dictionary where the keys are strings representing
        the bounding box IDs and the values are tuples containing the
        bounding box, keypoints, and descriptors for each bounding box.
        """
        return self.featuresPerBbox

    def getImageHeader(self) -> Header:
        """
        Get the header of the image

        :return Header object of the image captured in this reading
        """
        return self.imgHeader
