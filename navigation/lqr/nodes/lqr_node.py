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
from lqr import prepTrack, Track, SolverMatrices, setupMatrices, optimizeMinCurve, createRaceLine, pathToNumpy, addWidth

TRACK_WIDTH = rospy.get_param("/navigation/lqr/handler/track_width")
SAFETY_MARGIN = rospy.get_param("/navigation/lqr/handler/safety_margin")


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
