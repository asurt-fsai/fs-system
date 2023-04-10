#!/usr/bin/env python3
"""
lqr handler to integrate the following modules:
    - lqrPrepareTrack
    - lqr_optimize_track
"""
from typing import Tuple
import rospy
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from lqr.lqrPrepareTrack import prepTrack
from lqr.track import Track, SolverMatrices
from lqr.lqrOptimizeTrack import setupMatrices, optimizeMinCurve


TRACK_WIDTH = rospy.get_param("/navigation/lqr/handler/track_width")
SAFETY_MARGIN = rospy.get_param("/navigation/lqr/handler/safety_margin")


def normVecsToTrack(
    track: Track, alpha: npt.NDArray[np.float64]
) -> Tuple[npt.NDArray[np.float64], npt.NDArray[np.float64], npt.NDArray[np.float64],]:
    """
    Given a track's reference line and the optimal alpha values, this function
    calculates the upper and lower bounds of the track as well as the
    optimal raceline as XY coordinates.
    ----------
    track: Track
        Track class containing the track's reference line
    alphaOpt: np.ndarray
        Optimal alpha values for the track
    Returns
    -------
    trackUpBound: np.ndarray, shape=(n, 2)
        Upper bound of the track
    trackLowBound: np.ndarray, shape=(n, 2)
        Lower bound of the track
    trackRaceLine: np.ndarray, shape=(n, 2)
        Optimal raceline of the track
    """
    trackUpBound = track.smooth.track[:, :2] + track.smooth.normVectors * np.expand_dims(
        track.smooth.track[:, 2], 1
    )
    trackLowBound = track.smooth.track[:, :2] - track.smooth.normVectors * np.expand_dims(
        track.smooth.track[:, 3], 1
    )
    trackRaceLine = track.smooth.track[:, :2] + track.smooth.normVectors * np.expand_dims(alpha, 1)

    return (trackUpBound, trackLowBound, trackRaceLine)


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


def main() -> None:
    """
    Main Function of the lqrHandler
    """
    referenceTrack = np.array([])
    solverMatrices = SolverMatrices()
    rospy.init_node("lqr")
    rospy.loginfo("Waiting for Message")
    message = rospy.wait_for_message("/waypoints", Float64MultiArray, timeout=None)
    rospy.loginfo("Recieved Message")

    trackBuffer = np.array([message.data])
    noPoints = int(trackBuffer.shape[1] / 2)
    trackBuffer = np.stack((trackBuffer[0, :noPoints], trackBuffer[0, noPoints:]), axis=1)

    referenceTrack = addWidth(trackBuffer)

    trackClass = Track(referenceTrack)
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
    alphaOpt = optimizeMinCurve(trackClass, solverMatrices)

    upperBound, lowerBound, raceLine = normVecsToTrack(trackClass, alphaOpt)

    rate = rospy.Rate(10)
    raceLinePub = rospy.Publisher("raceline", Pose, queue_size=10)
    raceLineMsg = Pose()
    for i in range(len(raceLine)):
        raceLineMsg.position.x = raceLine[i, 0]
        raceLineMsg.position.y = raceLine[i, 1]
        raceLinePub.publish(raceLineMsg)
        rate.sleep()
    rospy.loginfo("Waypoints published")

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
