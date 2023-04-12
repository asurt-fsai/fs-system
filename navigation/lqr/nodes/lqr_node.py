#!/usr/bin/env python3
"""
lqr handler to integrate the following modules:
    - lqrPrepareTrack
    - lqr_optimize_track
"""
import rospy
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import tf2_ros
from nav_msgs.msg import Path
from lqr import prepTrack, Track, SolverMatrices, setupMatrices, optimizeMinCurve, createRaceLine

TRACK_WIDTH = rospy.get_param("/navigation/lqr/handler/track_width")
SAFETY_MARGIN = rospy.get_param("/navigation/lqr/handler/safety_margin")


def addWidth(trackNoWidth: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:

    """
    Adds width of track to track array if only the x and y coordinates of the track are given

    Parameters
    ----------
    trackNoWidth: np.ndarray, shape=(n, 2)
        Track array with only x and y coordinates

    Returns
    -------
    wideTrack: np.ndarray, shape=(n, 4)
        Track array with x, y, left width and right width
    """

    widthCol = np.ones(int(trackNoWidth.shape[0])) * (TRACK_WIDTH - SAFETY_MARGIN)
    trackNoWidth = np.stack(
        (
            trackNoWidth[:, 0],
            trackNoWidth[:, 1],
            widthCol / 2,
            widthCol / 2,
        ),
        axis=1,
    )
    wideTrack = np.reshape(trackNoWidth, (-1, 4))
    return wideTrack


def pathToNumpy(path: Path) -> npt.NDArray[np.float64]:
    """
    Converts a path message to a numpy array
    and transforms the coordinates to the world frame

    Parameters
    ----------
    path: Path
        Path message

    Returns
    -------
    pathArray: np.ndarray, shape=(n, 2)
        Numpy array with x and y coordinates of the path
    """
    tfBuffer = tf2_ros.Buffer()
    pathArray = np.zeros((len(path.poses), 2))
    for index, pose in enumerate(path.poses):
        globalPose = tfBuffer.transform(pose, "world", rospy.Duration(1))
        pathArray[index, 0] = globalPose.pose.position.x
        pathArray[index, 1] = globalPose.pose.position.y
    return pathArray


def main() -> None:
    """
    Main Function of the lqrHandler
    """
    referenceTrack = np.array([])
    solverMatrices = SolverMatrices()
    rospy.init_node("lqr")
    rospy.loginfo("Waiting for Message")
    message = rospy.wait_for_message("/waypoints", Path, timeout=None)
    rospy.loginfo("Recieved Message")

    trackBuffer = pathToNumpy(message)
    referenceTrack = addWidth(trackBuffer)
    trackClass = Track(referenceTrack)

    # Setup the track
    (
        trackClass.smooth.track,
        trackClass.smooth.normVectors,
        trackClass.smooth.alpha,
        trackClass.smooth.xCoeff,
        trackClass.smooth.yCoeff,
        trackClass.smooth.noPoints,
        trackClass.smooth.noSplines,
    ) = prepTrack(refTrack=trackClass.original)
    setupMatrices(trackClass, solverMatrices)

    # Calculate the optimal alpha values
    trackClass.optimized.alpha = optimizeMinCurve(trackClass, solverMatrices)

    # Calculate the track boundaries and raceline
    raceLineMsg = Path()
    raceLineMsg, raceLine, upperBound, lowerBound = createRaceLine(trackClass)

    # Publish the raceline
    raceLinePub = rospy.Publisher("raceline", Path, queue_size=10)
    raceLinePub.publish(raceLineMsg)
    rospy.loginfo("Waypoints published")

    # Plot
    if rospy.get_param("/navigation/lqr/handler/plot") == 1:
        plt.figure()
        plt.plot(raceLine[:, 0], raceLine[:, 1])
        plt.plot(trackClass.smooth.track[:, 0], trackClass.smooth.track[:, 1], "b--")
        plt.plot(upperBound[:, 0], upperBound[:, 1], "k-")
        plt.plot(lowerBound[:, 0], lowerBound[:, 1], "k-")
        plt.grid()
        plotArea = plt.gca()
        plotArea.set_aspect("equal", "datalim")
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
