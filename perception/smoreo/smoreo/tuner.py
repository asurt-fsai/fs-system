"""
Main ros node for the smoreo tuner
"""
#!/usr/bin/python3
import sys
# import rospy
# from tf_helper.StatusPublisher import StatusPublisher
# from smoreo.tuner.tunerServer import Tuner
import rclpy
from rclpy.node import Node
import os
from typing import Any, Union
import yaml
# from dynamic_reconfigure.server import Server
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from numpy.typing import NDArray
# from smoreo.cfg import smoreoConfig  # type: ignore

class TunerSystem(Node):
    def __init__(self):
        super().__init__("tuner")

        # self.srv: Server
        self.cutOffPublisher: rclpy.publisher.Publisher
        self.bridge: CvBridge
        self.lastImage: Union[NDArray[np.float32], None]

        # self.declare_parameters(
        #     namespace=" ",
        #     parameters=[
        #        ("smoreo/f", rclpy.Parameter.Type.DOUBLE),
        #         ("smoreo/cx", rclpy.Parameter.Type.DOUBLE),
        #         ("smoreo/cy", rclpy.Parameter.Type.DOUBLE),
        #         ("smoreo/cone_height", rclpy.Parameter.Type.DOUBLE),
        #         ("smoreo/camera_height_from_ground", rclpy.Parameter.Type.DOUBLE),
        #         ("smoreo/cut_off_y", rclpy.Parameter.Type.INTEGER),
               
        #     ],
        # )

        self.create_timer(0.1, self.timer_callback)
        # self.tunerStatus = StatusPublisher("/status/tuner")
        # self.tunerStatus.starting()
        # self.tuner = Tuner(self)
        self.start()
        # self.tunerStatus.ready()
       
    

    def parametersCallback(self, config, level: Any):
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
            cutOffy = self.get_parameter("smoreo/cut_off_y").get_parameter_value().integer_value
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
        # self.srv = Server(smoreoConfig, self.parametersCallback)
        self.cutOffPublisher = self.create_publisher(Image,
            self.get_parameter("/smoreo/cut_off_viz"), 10
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
                "f": TunerSystem.get_parameter("smoreo/f").get_parameter_value().double_value,
                "cx": TunerSystem.get_parameter("smoreo/cx").get_parameter_value().double_value,
                "cy": TunerSystem.get_parameter("smoreo/cy").get_parameter_value().double_value,
                "cone_height": TunerSystem.get_parameter("smoreo/cone_height").get_parameter_value().double_value,
                "camera_height_from_ground": TunerSystem.get_parameter("smoreo/camera_height_from_ground").get_parameter_value().double_value,
                "cut_off_y": TunerSystem.get_parameter("smoreo/cut_off_y").get_parameter_value().double_value,
            }
        }
        # Dump the parameters to a yaml file i the config folder
        yamlFile = r"../../../config/" + system + "Params.yaml"

        scriptDir = os.path.dirname(__file__)
        absFilePath = os.path.join(scriptDir, yamlFile)

        with open(absFilePath, "w", encoding="utf-8") as file:
            yaml.dump(params, file)
    def timer_callback(self):        
        self.visualizeCutOff()
        # self.tunerStatus.running()

def main(args = None) -> None:
    """
    Main Loop
    """

    rclpy.init(args = args)
    node = TunerSystem()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    print("Tuner Node")
    try:
        main()
    except rclpy.exceptions.ROSInterruptException:
        pass
    
    finally:
        try:
            assert len(sys.argv) > 2
        except Exception as exc:
            raise ValueError("Provide the name of the setup you are tuning") from exc
        TunerSystem.dumpParams(sys.argv[1])
