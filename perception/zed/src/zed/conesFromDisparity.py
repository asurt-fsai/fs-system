#!/usr/bin/python3

"""
Generate cones from disparity map
"""

from typing import Any, List, Dict, Union
import rospy
import numpy as np
import numpy.typing as npt
from stereo_msgs.msg import DisparityImage
from asurt_msgs.msg import LandmarkArray, Landmark
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import MarkerArray
from darknet_ros_msgs.msg import BoundingBoxes
from tf_helper.MarkerViz import MarkerViz
import message_filters


class ConesFromDisparity:
    """
    Class to subscribe to disparity image and bounding boxes and generate cones depth from disparity
    """

    def __init__(self, pixelsToConsider: int) -> None:
        """
        Constructor

        Parameters:
        ------------
        pixelsToConsider: int
            Number of pixels to consider for depth estimation, the current implementation
            uses the closest 'pixelsToConsider' pixels.

        Returns:
        -----------
        None
        """
        self.pixelsToConsider = pixelsToConsider
        self.markerViz = MarkerViz(coneHeight=0.2, coneRadius=0.1)
        self.camera: Dict[str, float] = {}
        self.disparityImage: DisparityImage = None
        self.conePub: rospy.Publisher
        self.markersPub: rospy.Publisher
        self.boundingBoxes: BoundingBoxes

    def disparityCallback(self, disparityImage: DisparityImage, bbx: BoundingBoxes) -> None:
        """
        Simple callback for disparity image and bounding boxes

        Parameters:
        ------------
        disparityImage: DisparityImage
            disparity image from ZED
        bbx: BoundingBoxes
            bounding boxes from darknet_ros

        Returns:
        -----------
        None
        """
        self.disparityImage = disparityImage
        self.boundingBoxes = bbx

    def getCameraParameters(self, cameraInfoTopic: str) -> None:
        """
        Get camera parameters from camera info topic

        Parameters:
        ------------
        cameraInfoTopic: str
            camera info topic name

        Returns:
        -----------
        None
        """
        cameraInfo = rospy.wait_for_message(cameraInfoTopic, CameraInfo)
        self.camera["cx"] = cameraInfo.K[2]
        self.camera["cy"] = cameraInfo.K[5]
        self.camera["f"] = cameraInfo.K[0]

    def start(
        self, disparityTopic: str, bboxTopic: str, coneTopic: str, cameraInfoTopic: str
    ) -> None:
        """
        Start the module by creating subscribers and publishers

        Parameters:
        ------------
        disparityTopic: str
            disparity topic name
        bboxTopic: str
            bounding boxes topic name
        coneTopic: str
            cone topic name
        cameraInfoTopic: str
            camera info topic name

        Returns:
        -----------
        None
        """
        disparitySub = message_filters.Subscriber(disparityTopic, DisparityImage)
        self.getCameraParameters(cameraInfoTopic)
        bboxSb = message_filters.Subscriber(bboxTopic, BoundingBoxes)
        timesync = message_filters.ApproximateTimeSynchronizer([disparitySub, bboxSb], 10, 0.1)
        timesync.registerCallback(self.disparityCallback)

        self.conePub = rospy.Publisher(coneTopic, LandmarkArray, queue_size=10)
        self.markersPub = rospy.Publisher(coneTopic + "_markers", MarkerArray, queue_size=10)

    def processBboxes(self, boundingBoxes: BoundingBoxes) -> npt.NDArray[np.float64]:
        """
        Process bounding boxes objects and return
        an np array representation.

        parameters
        ----------
        boundingBoxes: BoundingBoxes
            bounding boxes found in image
        Returns
        ------
        ndarray
        #boxes x 6 (#boxes,h,w,cy,cx,id,type)
        """
        bboxes = []
        for box in boundingBoxes.bounding_boxes:
            height = box.ymax - box.ymin
            width = box.xmax - box.xmin
            centerY = (box.ymax + box.ymin) // 2
            centerX = (box.xmax + box.xmin) // 2
            boxId = box.id
            classPob = box.probability
            if box.Class == "blue_cone":
                boxType = 0
            elif box.Class == "yellow_cone":
                boxType = 1
            elif box.Class == "orange_cone":
                boxType = 2
            elif box.Class == "large_cone":
                boxType = 3
            else:
                boxType = 4
            bboxes.append(
                [int(height), int(width), int(centerY), int(centerX), boxId, boxType, classPob]
            )
        return np.asarray(bboxes)

    def toLandmarkArray(self, cones: List[List[Any]]) -> LandmarkArray:
        """
        Convert a list of cones to a LandmarkArray message

        Parameters:
        ------------
        cones: List[Tuple[float,float,float,float,float,float]]
            list of cones (x,y,z,id,type,probability)

        Returns:
        -----------
        LandmarkArray

        """
        landmarks = LandmarkArray()
        landmarks.header = self.disparityImage.header
        for cone in cones:
            landmark = Landmark()
            landmark.position.x = cone[0]
            landmark.position.y = cone[1]
            landmark.position.z = cone[2]
            landmark.identifier = cone[3]
            landmark.type = cone[4]
            landmark.probability = cone[5]
            landmarks.landmarks.append(landmark)
        return landmarks

    def run(self) -> Union[LandmarkArray, None]:
        """
        Run the module and publish cones

        Parameters:
        ------------
        None

        Returns:
        -----------
        LandmarkArray

        """
        if self.disparityImage is None:
            return None
        disparity = np.frombuffer(self.disparityImage.image.data, dtype=np.float32).reshape(
            self.disparityImage.image.height, self.disparityImage.image.width
        )
        # Convert the disparities to the correct range
        disparity = np.nan_to_num(disparity)
        bboxes = self.processBboxes(self.boundingBoxes)
        cones = []
        for box in bboxes:
            # height, width, centerY, centerX, ID, Type, Probability = box
            ymin = int(box[2] - box[0] // 2)
            ymax = int(box[2] + box[0] // 2)
            xmin = int(box[3] - box[1] // 4)
            xmax = int(box[3] + box[1] // 4)
            # Get a strip in the middle of the bbox with width 20% of bbox width
            disparityBox = disparity[ymin:ymax, xmin:xmax]
            disparityBox = disparityBox.flatten()
            disparityBox = disparityBox[disparityBox != 0]
            disparityBox = np.sort(disparityBox)
            if len(disparityBox) > self.pixelsToConsider:
                disparityBox = disparityBox[: self.pixelsToConsider]
            if len(disparityBox) == 0 or np.isnan(disparityBox[0]):
                continue
            disparityBox = np.mean(disparityBox)

            zCord = self.disparityImage.f * self.disparityImage.T / disparityBox
            xCord = (((xmin + xmax) / 2) - self.camera["cx"]) * zCord / self.disparityImage.f
            yCord = (((ymin + ymax) / 2) - self.camera["cy"]) * zCord / self.disparityImage.f
            # Convert from image Frame to Camera Frame and append id,type and probability
            cones.append([zCord, -xCord, -yCord, box[4], box[5], box[6]])
        landmarks = self.toLandmarkArray(cones)
        self.conePub.publish(landmarks)
        self.markersPub.publish(self.markerViz.conesToMarkers(landmarks))
        return landmarks
