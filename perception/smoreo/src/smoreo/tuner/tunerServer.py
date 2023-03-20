"""
Smoreo tuner class
"""
import os
from typing import Any, Union
import yaml
import rospy
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from numpy.typing import NDArray
from smoreo.cfg import smoreoConfig  # type: ignore


class Tuner:
    """
    Tuner class for smoreo
    """

    def __init__(self) -> None:
        self.srv: Server
        self.cutOffPublisher: rospy.Publisher
        self.bridge: CvBridge
        self.lastImage: Union[NDArray[np.float32], None]

    # pylint: disable=unused-argument
    def parametersCallback(self, config: smoreoConfig, level: Any) -> smoreoConfig:
        """
        Callback for dynamic reconfigure server
        Parameters:
        ------------
        config: smoreo_tunerConfig
        """

        rospy.set_param("smoreo/f", config["f"])
        rospy.set_param("smoreo/cx", config["cx"])
        rospy.set_param("smoreo/cy", config["cy"])
        rospy.set_param("smoreo/cone_height", config["cone_h"])
        rospy.set_param("smoreo/camera_height_from_ground", config["camera_h"])
        rospy.set_param("smoreo/cut_off_y", config["cut_off_y"])
        return config

    def visualizeCutOff(self) -> None:
        """
        Visualize the cut off line
        """
        if self.lastImage is not None:
            cutOffy = rospy.get_param("smoreo/cut_off_y")
            startPoint = (0, cutOffy)
            endPoint = (self.lastImage.shape[1], cutOffy)
            out = cv.line(self.lastImage.copy(), startPoint, endPoint, (0, 0, 0), 10)
            out = self.bridge.cv2_to_imgmsg(out, encoding="passthrough")
            self.cutOffPublisher.publish(out)

    def imageCallback(self, image: Image) -> None:
        """
        Callback for image subscriber

        Parameters:
        ------------
        image: Image
        """
        image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.lastImage = np.array(image)

    def start(self) -> None:
        """
        Start the node
        """
        if not rospy.has_param("/smoreo/camera_raw") or not rospy.has_param("/smoreo/cut_off_viz"):
            raise ValueError(
                "smoreo: ensure all the required topics for tuner are provided\n \
                             -/smoreo/camera_raw\n \
                             -/smoreo/cut_off_viz."
            )
        self.srv = Server(smoreoConfig, self.parametersCallback)
        self.cutOffPublisher = rospy.Publisher(
            rospy.get_param("/smoreo/cut_off_viz"), Image, queue_size=10
        )
        self.bridge = CvBridge()
        self.lastImage = None
        rospy.Subscriber(rospy.get_param("/smoreo/camera_raw"), Image, self.imageCallback)

    @staticmethod
    def dumpParams(system: str) -> None:
        """
        Dump the parameters to a yaml file
        """
        # Get the parameters
        params = {
            "smoreo": {
                "f": rospy.get_param("smoreo/f"),
                "cx": rospy.get_param("smoreo/cx"),
                "cy": rospy.get_param("smoreo/cy"),
                "cone_height": rospy.get_param("smoreo/cone_height"),
                "camera_height_from_ground": rospy.get_param("smoreo/camera_height_from_ground"),
                "cut_off_y": rospy.get_param("smoreo/cut_off_y"),
            }
        }
        # Dump the parameters to a yaml file i the config folder
        yamlFile = r"../../../config/" + system + "Params.yaml"

        scriptDir = os.path.dirname(__file__)
        absFilePath = os.path.join(scriptDir, yamlFile)

        with open(absFilePath, "w", encoding="utf-8") as file:
            yaml.dump(params, file)
