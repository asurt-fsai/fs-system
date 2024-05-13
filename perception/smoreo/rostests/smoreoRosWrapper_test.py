"""
Test file for smoreoRosWrapper
"""
import unittest
import time
from smoreo.smoreo_system import SmoreoSystem
from typing import Dict, Any
import rclpy
import numpy as np
import numpy.typing as npt
import tf
from sensor_msgs.msg import CameraInfo
from asurt_msgs.msg import LandmarkArray
from smoreo.smoreo import Smoreo


class SmoreoRosWrappertest(unittest.TestCase):
    """
    Test class for smoreoRosWrapper
    """

    def setUp(self) -> None:
        self.params = {"f": 100, "cx": 200, "cy": 300, "cone_height": 0.5}
     

    def testParams(self) -> None:
        """
        Test if the params are passed correctly
        """
        rclpy.init()
        smoreoRosWrapper = SmoreoSystem()

        node = rclpy.create_node("test_node")
        # self.assertEqual(smoreoRosWrapper.params, self.params)

        node.declare_parameter("/smoreo/hardcode_params", False)
        node.declare_parameter("/smoreo/camera_info", "/camera_info")
        # node.set_parameters([rclpy.parameter.Parameter("smoreo.hardcode_params", rclpy.ParameterType.BOOL, True)])
        # node.set_parameters([rclpy.parameter.Parameter("smoreo.camera_info", rclpy.ParameterType.STRING, "/camera_info")])
        
        with self.assertRaises(TypeError):
            smoreoRosWrapper.getParams()
     
        cameraInfoMsg = CameraInfo()
        cameraInfoMsg.k = [100.0, 0.0, 200.0, 0.0, 100.0, 300.0, 0.0, 0.0, 1.0]
        time.sleep(0.1)
        pub = node.create_publisher( CameraInfo, "/camera_info",10)
        pub.publish(cameraInfoMsg)

        rclpy.spin_once(node, timeout_sec=0.1)
        # self.assertEqual(smoreoRosWrapper.getParams()["f"], 336.0)
        # self.assertEqual(smoreoRosWrapper.getParams()["cx"], 336.0)
        # self.assertEqual(smoreoRosWrapper.getParams()["cy"], 188.0)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    unittest.main()