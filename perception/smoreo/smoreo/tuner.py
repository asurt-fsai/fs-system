"""
Main ros node for the smoreo tuner
"""
#!/usr/bin/python3
import sys
# import rospy
from tf_helper.StatusPublisher import StatusPublisher
from smoreo.tuner.tunerServer import Tuner
import rclpy
from rclpy.node import Node
import os
from typing import Any, Union
import yaml
from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from numpy.typing import NDArray
from smoreo.cfg import smoreoConfig  # type: ignore

class TunerSystem(Node):
    def __init__(self):
        super().__init__("tuner")

        self.srv: Server
        self.cutOffPublisher: rclpy.publisher
        self.bridge: CvBridge
        self.lastImage: Union[NDArray[np.float32], None]



        self.create_timer(0.1, self.timer_callback)
        self.tunerStatus = StatusPublisher("/status/tuner")
        self.tunerStatus.starting()
        # self.tuner = Tuner(self)
        self.start()
        self.tunerStatus.ready()
       
    def timer_callback(self):        
        self.visualizeCutOff()
        self.tunerStatus.running()

    def parametersCallback(self, config: smoreoConfig, level: Any) -> smoreoConfig:
        """
        Callback for dynamic reconfigure server
        Parameters:
        ------------
        config: smoreo_tunerConfig
        """

        self.declare_parameter("smoreo/f", config["f"])
        self.declare_parameter("smoreo/cx", config["cx"])
        self.declare_parameter("smoreo/cy", config["cy"])
        self.declare_parameter("smoreo/cone_height", config["cone_h"])
        self.declare_parameter("smoreo/camera_height_from_ground", config["camera_h"])
        self.declare_parameter("smoreo/cut_off_y", config["cut_off_y"])
        return config

    def visualizeCutOff(self) -> None:
        """
        Visualize the cut off line
        """
        if self.lastImage is not None:
            cutOffy = self.get_parameter("smoreo/cut_off_y")
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
        if not self.has_parameter("/smoreo/camera_raw") or not self.has_parameter("/smoreo/cut_off_viz"):
            raise ValueError(
                "smoreo: ensure all the required topics for tuner are provided\n \
                             -/smoreo/camera_raw\n \
                             -/smoreo/cut_off_viz."
            )
        self.srv = Server(smoreoConfig, self.parametersCallback)
        self.cutOffPublisher = self.create_publisher(Image,
            self.get_parameter("/smoreo/cut_off_viz"), queue_size=10
        )
        self.bridge = CvBridge()
        self.lastImage = None
        self.subscription = self.create_subscription(Image,self.get_parameter("/smoreo/camera_raw"), self.imageCallback)

    @staticmethod
    def dumpParams(system: str) -> None:
        """
        Dump the parameters to a yaml file
        """
        # Get the parameters
        params = {
            "smoreo": {
                "f": Tuner.node.get_parameter("smoreo/f"),
                "cx": Tuner.node.get_parameter("smoreo/cx"),
                "cy": Tuner.node.get_parameter("smoreo/cy"),
                "cone_height": Tuner.node.get_parameter("smoreo/cone_height"),
                "camera_height_from_ground": Tuner.node.get_parameter("smoreo/camera_height_from_ground"),
                "cut_off_y": Tuner.node.get_parameter("smoreo/cut_off_y"),
            }
        }
        # Dump the parameters to a yaml file i the config folder
        yamlFile = r"../../../config/" + system + "Params.yaml"

        scriptDir = os.path.dirname(__file__)
        absFilePath = os.path.join(scriptDir, yamlFile)

        with open(absFilePath, "w", encoding="utf-8") as file:
            yaml.dump(params, file)



def main(args = None) -> None:
    """
    Main Loop
    """
  
    rclpy.init(args = args)
    node = TunerSystem()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        try:
            assert len(sys.argv) > 2
        except Exception as exc:
            raise ValueError("Provide the name of the setup you are tuning") from exc
        Tuner.dumpParams(sys.argv[1])
