"""
indication for the interpreter that should be used to run the script.
"""
# pylint: disable=global-variable-not-assigned
# pylint: disable=global-statement
from typing import List
import rospy
import numpy as np
import numpy.typing as npt
from asurt_msgs.msg import LandmarkArray
from tf_helper.utils import parseLandmarks


LISTDIS: List[float] = []
GTLANDMARKS: npt.NDArray[np.float64] = np.array([])
rospy.init_node("results_calculator")


def gtCallback(data: LandmarkArray) -> None:
    """
    inputs : data(ground truth landmarks)
    outputs : GTLANDMARKS
    function : save the ground truth landmarks in a global list
    """
    global GTLANDMARKS
    GTLANDMARKS = parseLandmarks(data)


def conesCallback(data: LandmarkArray) -> None:
    """
    inputs : GTLANDMARKS, data(detected cones)
    outputs : the difference in position between GTLANDMARKS and detected cones
    function : compute and save the differences in a global list
    the difference should be computed between each landmark in detectedCones
    and the closest landmark in GTLANDMARKS
    """
    global LISTDIS
    detectedCones = parseLandmarks(data)
    diff = []
    if not GTLANDMARKS:
        return
    for cone in detectedCones:
        distances = np.linalg.norm(GTLANDMARKS[:, :2] - cone[:2].reshape(1, 2), axis=1)
        diff.append(np.min(distances))
    avgDiff = np.mean(diff)
    LISTDIS.append(avgDiff)
    np.save("./results.npy", LISTDIS)


if __name__ == "__main__":
    rospy.Subscriber("/ground_truth/landmarks", LandmarkArray, gtCallback)
    rospy.Subscriber("/cones_map", LandmarkArray, conesCallback)
    rospy.spin()
