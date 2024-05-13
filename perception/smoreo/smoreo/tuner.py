#!/usr/bin/python3
import sys
from tf_helper.StatusPublisher import StatusPublisher
# from smoreo.tuner.tunerServer import Tuner
import rclpy
from rclpy.node import Node
from rclpy.parameter import ParameterType
from interfaces.srv import SmoreoSrv
import os
from typing import Any, Union
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import yaml
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
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/smoreo/camera_raw', rclpy.Parameter.Type.STRING),
                ('/smoreo/cut_off_viz', rclpy.Parameter.Type.STRING),
                ('smoreo/f',rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/cx',rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/cy',rclpy.Parameter.Type.DOUBLE),
                ('/physical/cone_height',rclpy.Parameter.Type.DOUBLE),
                ('/physical/camera_height_from_ground',rclpy.Parameter.Type.DOUBLE),
                ('/smoreo/cut_off_y',rclpy.Parameter.Type.INTEGER)
            ]
            )
        self.srv = self.create_service(SmoreoSrv, '/smoreo/camera_raw', self.parametersCallback)
        self.create_timer(0.1, self.timer_callback)
        self.tunerStatus = StatusPublisher("/status/tuner",self)
        self.tunerStatus.starting()
        self.start()
        self.tunerStatus.ready()
       
    

    def parametersCallback(self, request, response):
        # Process the parameters
        self.get_logger().info("Received parameters request")
        self.get_parameter('f').get_parameter_value().double_value = response.f 
        self.get_parameter('cx').get_parameter_value().double_value=  response.cx
        self.get_parameter('cy').get_parameter_value().double_value=  response.cy 
        self.get_parameter('cone_h').get_parameter_value().double_value= response.cone_h
        self.get_parameter('camera_h').get_parameter_value().double_value=response.camera_h 
        self.get_parameter('cut_off_y').get_parameter_value().integer_value=response.cut_off_y
        return response

    def visualizeCutOff(self) -> None:
        """
        Visualize the cut off line
        """
        if self.lastImage is not None:
            cutOffy = self.get_parameter("/smoreo/cut_off_y").get_parameter_value().integer_value
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
        self.get_logger().info("Received image")
        image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.lastImage = np.array(image)

    def start(self) -> None:
        """
        Start the node
        """
        self.get_logger().info("Starting Tuner Node")
        
        if not self.has_parameter("/smoreo/camera_raw") or not self.has_parameter("/smoreo/cut_off_viz"):
            raise ValueError(
                "smoreo: ensure all the required topics for tuner are provided\n \
                             -/smoreo/camera_raw\n \
                             -/smoreo/cut_off_viz."
            )
      
        self.cutOffPublisher = self.create_publisher(Image,
            self.get_parameter("/smoreo/cut_off_viz").get_parameter_value().string_value, 10
        )
        self.bridge = CvBridge()
        self.lastImage = None
        self.subscription = self.create_subscription(Image,self.get_parameter("/smoreo/camera_raw").get_parameter_value().string_value, self.imageCallback, 10)

    @staticmethod
    def dumpParams(system: str) -> None:
        """
        Dump the parameters to a yaml file
        """
        # Get the parameters
        params = {
            "smoreo": {
                "f": TunerSystem.get_parameter("/smoreo/f").get_parameter_value().double_value,
                "cx": TunerSystem.get_parameter("/smoreo/cx").get_parameter_value().double_value,
                "cy": TunerSystem.get_parameter("/smoreo/cy").get_parameter_value().double_value,
                "cone_height": TunerSystem.get_parameter("/physical/cone_height").get_parameter_value().double_value,
                "camera_height_from_ground": TunerSystem.get_parameter("/physical/camera_height_from_ground").get_parameter_value().double_value,
                "cut_off_y": TunerSystem.get_parameter("/smoreo/cut_off_y").get_parameter_value().double_value,
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
        self.tunerStatus.running()

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
