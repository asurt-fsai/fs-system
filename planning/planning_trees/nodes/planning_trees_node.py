"""
Planning trees main node to interface with the cpp node
"""
import rospy
import numpy as np
import numpy.typing as npt
from nav_msgs.msg import Path
from asurt_msgs.msg import LandmarkArray
from tf_helper.TFHelper import TFHelper
from tf_helper.utils import parsePathMessage, createPathMessage

from scipy import interpolate


def conesCallback(msg: LandmarkArray, frameId: str, pub: rospy.Publisher, helper: TFHelper) -> None:
    """
    Callback for cones topic

    Parameters
    ----------
    msg : LandmarkArray
        Message from cones topic
    frameId : str
        Frame ID of the message
    pub : rospy.Publisher
        Publisher for the transformed message
    helper : TFHelper
        TFHelper object
    """
    transformedMsg = helper.transformMsg(msg, frameId)
    pub.publish(transformedMsg)


def fitSpline(waypoints: npt.NDArray[np.float64], nSamples: int = 20) -> npt.NDArray[np.float64]:
    """
    Fits a spline to the given waypoint list

    Parameters
    ----------
    waypoints: np.array, shape = [N,2]
        Waypoints to fit the spline to
    nSamples: int
        Number of points to sample from the spline

    Returns
    -------
    np.array, shape = (nSamples, 2)
        Points sampled from the fitted spline
    """
    if waypoints.shape[0] < 2:
        return waypoints

    degree = 2
    if waypoints.shape[0] == 2:
        degree = 1

    tck = interpolate.splprep(waypoints.T, w=np.ones(waypoints.shape[0]), s=20, k=degree)[0]
    unew = np.linspace(0, 1, nSamples)
    newWaypoints = np.array(interpolate.splev(unew, tck)).T
    return newWaypoints


def filterAndPub(msg: Path, enableSpline: bool, waypointsPub: rospy.Publisher) -> None:
    """
    If filter is enabled, fits a spline to the waypoints before publishing them

    Parameters
    ----------
    msg: Path
        Path message recieved
    enableSpline: bool
        If true, a spline is fitted
    waypointsPub: rospy.Publisher
        Publisher to publish the waypoints to
    """
    waypoints = parsePathMessage(msg)
    if enableSpline:
        waypoints = fitSpline(waypoints)
    pathMsg = createPathMessage(waypoints, msg.header.frame_id)
    waypointsPub.publish(pathMsg)


def main() -> None:
    """
    Main function
    """
    rospy.init_node("planning_trees_node")
    helper = TFHelper("planning_trees_node")

    conesTopic = rospy.get_param("/planning/cones_topic")
    frameId = rospy.get_param("/planning/frame_id")
    cppConesTopic = rospy.get_param("/planning/cpp_cones_topic")
    cppOutConesTopic = rospy.get_param("/planning/cpp_out_cones_topic")
    waypointsTopic = rospy.get_param("/planning/waypoints_topic")
    enableSpline = rospy.get_param("/planning/enable_spline")

    cppPub = rospy.Publisher(cppConesTopic, LandmarkArray, queue_size=10)
    waypointsPub = rospy.Publisher(waypointsTopic, Path, queue_size=10)

    rospy.Subscriber(
        conesTopic, LandmarkArray, lambda data: conesCallback(data, frameId, cppPub, helper)
    )
    rospy.Subscriber(
        cppOutConesTopic, Path, lambda data: filterAndPub(data, enableSpline, waypointsPub)
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
