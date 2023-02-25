"""
Smoreo ros wrapper
"""
from typing import Dict, Any
import rospy
import numpy as np
import numpy.typing as npt
import tf
from sensor_msgs.msg import CameraInfo
from asurt_msgs.msg import LandmarkArray
from smoreo.smoreo import Smoreo
from darknet_ros_msgs.msg import BoundingBoxes


class SmoreoRosWrapper:
    """
    Ros wrapper for smoreo system
    """

    def __init__(self) -> None:
        self.params: Dict[str, Any]
        self.landmarkPub: rospy.Publisher
        self.boundingBoxes: BoundingBoxes
        self.smoreo: Smoreo
    def getParams(self) -> Dict[str, Any]:
        """
        Get params of the system from the ros parameters server

        Parameters:
        ------------
        None
        Returns:
        -----------
        Dict[str,Any]: parameters obtained from the ros parameter server.
        """
        if rospy.get_param("/smoreo/hardcode_params"):
            fInPixels = rospy.get_param("smoreo/f_in_pixels")
            cameraCx = rospy.get_param("smoreo/cx")
            camerCy = rospy.get_param("smoreo/cy")
        else:
            cameraInfo = rospy.wait_for_message(rospy.get_param("/smoreo/camera_info"), CameraInfo)
            fInPixels = cameraInfo.K[0]
            cameraCx = cameraInfo.K[2]
            camerCy = cameraInfo.K[5]

        coneHeight = rospy.get_param("smoreo/cone_height", 0.4)

        listener = tf.TransformListener()
        listener.waitForTransform(
            "/worldcoordinates", "/cameracoordinates", rospy.Time(), rospy.Duration(4.0)
        )
        _, worldToCameraRotation = listener.lookupTransform(
            "/worldcoordinates", "/cameracoordinates", rospy.Time(0)
        )
        params = {
            "cx": cameraCx,
            "cy": camerCy,
            "f": fInPixels,
            "k": np.array(
                [[fInPixels, 0.0, cameraCx], [0.0, fInPixels, camerCy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            ),
            "worldCords_inCamera": np.array(
                tf.transformations.quaternion_matrix(worldToCameraRotation)[:3, :3], dtype=int
            ),
            "camera_height_from_ground": rospy.get_param("smoreo/camera_height", 0.5),
            "cut_off_y": rospy.get_param("smoreo/cut_off_y"),
            "cone_height": coneHeight,
        }
        return params

    def processBboxes(self, boundingBoxes: BoundingBoxes) -> npt.NDArray[np.float64]:
        """
        Process bounding boxes objects and return
        an np array representation.

        parameters
        ----------
        bboxes2: boundingBoxes
        bounding boxes found in image
        Returns
        ------
        ndarray
        #boxes x 5 (#boxes,h,w,cy,cx,id)
        """
        bboxes = []
        for box in boundingBoxes:
            height = box.ymax - box.ymin
            width = box.xmax - box.xmin
            centerY = (box.ymax + box.ymin) // 2
            centerX = (box.xmax + box.xmin) // 2
            boxId = box.id
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
            bboxes.append([height, width, centerY, centerX, boxId, boxType])
        return np.asarray(bboxes)

    def createPublishers(self) -> None:
        """
        Create needed publishers
        """
        self.landmarkPub = rospy.Publisher(
            rospy.get_param("/smoreo/predicted_landmarks"), LandmarkArray, queue_size=10
        )

    def setBoundingBoxes(self, boundingBoxes: BoundingBoxes) -> None:
        """
        Main callback function for the system that accepts bounding boxes from the subscriber.

        Parameters:
        ------------
        boundingBoxes: BoundingBoxes

        Returns:
        -----------
        None
        """
        self.boundingBoxes = self.processBboxes(boundingBoxes.bounding_boxes)

    def run(self) -> None:
        """
        Run the system.
        """
        _, predictedLandmarks = self.smoreo.predictWithBase(self.boundingBoxes)
        self.landmarkPub.publish(predictedLandmarks)

    def start(self) -> None:
        """
        Run the system.
        """
        self.createPublishers()
        self.params = self.getParams()
        self.smoreo = Smoreo(self.params)
        rospy.Subscriber(
            rospy.get_param("/smoreo/bounding_boxes"), BoundingBoxes, self.setBoundingBoxes
        )
