"""
Moreo System
"""
# pylint: disable=import-error, no-name-in-module
from typing import Dict, Any
import numpy as np
import rospy
from asurt_msgs.msg import LandmarkArray
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import message_filters
import tf
from Base.moreo_base import MoreoBase
from utils.reading import Reading
from utils.buffer_manager import BufferManager
from utils.visualizer import Visualizer
from moreo.FEX.oldDistributedFEX import (
    DistributedFeatureDetectionSystem,
)
import np.typing as npt
from darknet_ros_msgs.msg import BoundingBoxes


class MoreoSystem:
    """
    This is a module that runs and manages moreo.
    """

    def __init__(self) -> None:
        """
        Initializes the MoreoSystem class by setting the moreo attribute,
        initializing the flags for visualization and reprojection testing,
        and creating the BufferManager and publishers.
        """
        self.moreo: MoreoBase
        self.visualize = rospy.get_param("moreo/visualize_process", False)
        self.reprojectionTesting = rospy.get_param("moreo/visualize_reprojection", False)
        self.params: Dict[str, Any]
        self.bufferManager = BufferManager(
            rospy.get_param("moreo/bufferSize", 5), DistributedFeatureDetectionSystem()
        )
        self.publishers: Dict[str, rospy.Publisher]
        self.visualizer: Visualizer

    def findParams(self) -> None:
        """
        Returns a dictionary containing the parameters for the Moreo system.
        These parameters are obtained by getting the values from the ROS parameters
        server and performing transformations on them.
        """
        focalLength = rospy.get_param("f_in_pixels")
        cameraCx = rospy.get_param("cx")
        cameraCy = rospy.get_param("cy")

        listener = tf.TransformListener()
        listener.waitForTransform("/Obj_F", "/front_camera_rgb", rospy.Time(), rospy.Duration(4.0))
        listener.waitForTransform(
            "/worldcoordinates", "/cameracoordinates", rospy.Time(), rospy.Duration(4.0)
        )

        trans, rot = listener.lookupTransform("/Obj_F", "/front_camera_rgb", rospy.Time(0))
        _, worldToCameraRot = listener.lookupTransform(
            "/worldcoordinates", "/cameracoordinates", rospy.Time(0)
        )
        # print(rot,tf.transformations.euler_from_quaternion(rot))
        self.params = {
            "camera_translation_vector": np.array(
                [[trans[0]], [trans[1]], [trans[2]]], dtype=np.float64
            ),
            "camera_rotation_matrix": np.asarray(tf.transformations.quaternion_matrix(rot))[:3, :3],
            "k": np.array(
                [
                    [focalLength, 0.0, cameraCx],
                    [0.0, focalLength, cameraCy],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            ),
            "worldCords_inCamera": np.array(
                tf.transformations.quaternion_matrix(worldToCameraRot)[:3, :3],
                dtype=int,
            ),
        }
        # print(self.params["camera_translation_vector"],
        # self.params["camera_rotation_matrix"],self.params["worldCords_inCamera"])

    def processBboxes(self, boundingBoxes: BoundingBoxes) -> npt.NDArray[np.float64]:
        """
        Processes a `BoundingBoxes` message and returns an numpy array
        containing the processed bounding boxes.

        Parameters:
        ----------
        boundingBoxes: (BoundingBoxes)
            A `BoundingBoxes` message to be processed.

        Returns:
        --------
        npt.NDArray[(,6), np.float64]
            A numpy array containing the processed bounding boxes.
        """
        bboxes = []
        for box in boundingBoxes.bounding_boxes:
            height = box.ymax - box.ymin
            width = box.xmax - box.xmin
            centerY = (box.ymax + box.ymin) // 2
            centerX = (box.xmax + box.xmin) // 2
            boxId = box.id
            if box.Class == "blue_cone":
                bType = 0
            elif box.Class == "yellow_cone":
                bType = 1
            elif box.Class == "orange_cone":
                bType = 2
            elif box.Class == "large_cone":
                bType = 3
            else:
                bType = 4
            bboxes.append([height, width, centerY, centerX, boxId, bType])
        return np.asarray(bboxes)

    def createPublishers(self) -> None:
        """
        Creates publishers for the Moreo system by publishing to topics
        specified in the ROS parameters server.
        """
        self.publishers["newBboxPub"] = rospy.Publisher(
            rospy.get_param("/moreo/update_bbox"), BoundingBoxes, queue_size=10
        )
        self.publishers["landmarkPub"] = rospy.Publisher(
            rospy.get_param("/moreo/predicted_landmarks"), LandmarkArray, queue_size=10
        )
        self.publishers["markerArrayPub"] = rospy.Publisher(
            rospy.get_param("/moreo/visualize_poses"), MarkerArray, queue_size=10
        )
        visuals = {}
        visuals["epilines"] = rospy.Publisher(
            rospy.get_param("/moreo/epilines_topic"), Image, queue_size=10
        )
        visuals["matches"] = rospy.Publisher(
            rospy.get_param("/moreo/matches_topic"), Image, queue_size=10
        )
        visuals["reprojection"] = rospy.Publisher(
            rospy.get_param("/moreo/reprojection_topic"), Image, queue_size=10
        )
        self.visualizer = Visualizer(visuals)

    def recievDataCallback(self, img: Image, boundingBoxes: BoundingBoxes) -> None:
        """
        Callback to recieve the data from the synchronizer and store it in buffer

        Parameters:
        ----------
        img: Image
            Obtained image message
        boundingBoxes: BoundingBoxes
            obtained BoundingBoxes message which is a list of BoundingBox objects
        """
        odom = rospy.wait_for_message("/odometry", Odometry)
        self.bufferManager.addToBuffer(img, odom, self.processBboxes(boundingBoxes))

    def startMoreo(self) -> None:
        """
        Start running moreo
        """
        imageSub = message_filters.Subscriber("/front_camera_rgb/image_raw", Image)
        boundingBoxes = message_filters.Subscriber(
            "/darknet_ros/bounding_boxes_filtered", BoundingBoxes
        )

        timeSynch = message_filters.TimeSynchronizer([imageSub, boundingBoxes], 20)
        timeSynch.registerCallback(self.recievDataCallback)
        self.createPublishers()
        self.findParams()
        self.moreo = MoreoBase(self.params, self.visualizer)
        while not rospy.is_shutdown():
            prevReading, curReading = self.bufferManager.getPair(0.3)
            if isinstance(prevReading, Reading) and isinstance(curReading, Reading):
                started = rospy.get_rostime()
                landmarks = self.moreo.reconstruct(prevReading, curReading)
                now = rospy.get_rostime()
                print(
                    "Need time was: "
                    + str((now.secs - started.secs) * 1000 + (now.nsecs - started.nsecs) / 1000000)
                )
                self.publishers["landmarkPub"].publish(landmarks)
