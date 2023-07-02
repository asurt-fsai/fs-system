"""
indication for the interpreter that should be used to run the script.
"""
# pylint: disable=global-variable-not-assigned
# pylint: disable=global-statement
import rospy
import numpy as np
from asurt_msgs.msg import LandmarkArray
from tf_helper.MarkerViz import MarkerViz
from tf_helper.utils import parseLandmarks, createLandmarkMessage
from visualization_msgs.msg import MarkerArray

rospy.init_node("noise_publisher")


def gtCallback(data: LandmarkArray) -> None:
    """
    inputs : data(ground truth landmarks)
    outputs : GT
    function : save the ground truth landmarks in a global list
    """
    global GTLANDMARKS
    groundTruth = parseLandmarks(data)
    noiseCones = groundTruth[:, :2] + np.random.normal(0, 0.1, (groundTruth[:, :2].shape))
    toPub = createLandmarkMessage(noiseCones, groundTruth[:, 2], "velodyne")
    landmark_pub.publish(toPub)
    gtVisual = marker_viz.conesToMarkers(toPub)
    marker_pub.publish(gtVisual)


marker_viz = MarkerViz(0.2, 0.4)
rospy.Subscriber("/ground_truth/landmarks", LandmarkArray, gtCallback)
landmark_pub = rospy.Publisher("/gt_map", LandmarkArray, queue_size=1)
marker_pub = rospy.Publisher("/gt_map_visualization", MarkerArray, queue_size=1)


rospy.spin()
