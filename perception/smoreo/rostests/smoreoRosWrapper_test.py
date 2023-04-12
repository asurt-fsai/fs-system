"""
Test file for smoreoRosWrapper
"""
import unittest
from smoreo.smoreo import SmoreoRosWrapper
from typing import Dict, Any
import rospy
import numpy as np
import numpy.typing as npt
import tf
from sensor_msgs.msg import CameraInfo
from asurt_msgs.msg import LandmarkArray
from smoreo.smoreo import Smoreo
from darknet_ros_msgs.msg import BoundingBoxes


class SmoreoRosWrappertest(unittest.TestCase):
    """
    Test class for smoreoRosWrapper
    """

    def setUp(self) -> None:
        self.params = {"f_in_pixels": 100, "cx": 200, "cy": 300, "cone_height": 0.5}

    def testParams(self) -> None:
        """
        Test if the params are passed correctly
        """
        smoreoRosWrapper = SmoreoRosWrapper()
        rospy.set_param("/smoreo/hardcode_params", True)
        self.assertEqual(smoreoRosWrapper.params, self.params)
        rospy.set_param("/smoreo/hardcode_params", False)
        rospy.set_param("/smoreo/camera_info", "/camera_info")
        with self.assertRaises(TypeError):
            smoreoRosWrapper.params()
        rospy.set_param("/smoreo/hardcode_params", False)
        rospy.set_param("/smoreo/camera_info", "/camera_info")
        cameraInfoMsg = CameraInfo()
        cameraInfoMsg.K = [100, 0, 200, 0, 100, 300, 0, 0, 1]
        rospy.sleep(0.1)
        pub = rospy.Publisher("/camera_info", CameraInfo, queue_size=10)
        pub.publish(cameraInfoMsg)
        self.assertEqual(smoreoRosWrapper.params["f_in_pixels"], 100)
        self.assertEqual(smoreoRosWrapper.params["cx"], 200)
        self.assertEqual(smoreoRosWrapper.params["cy"], 300)
