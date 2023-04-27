#!/usr/bin/env python
"""
indication for the interpreter that should be used to run the script.
"""
import rospy
from nav_msgs.msg import Odometry
from asurt_msgs.msg import Roadstate, LandmarkArray
from tf_helper.MarkerViz import MarkerViz
from tf_helper.utils import parseLandmarks, createLandmarkMessage
from visualization_msgs.msg import MarkerArray
from supervisor_data.src.supervisor_data_number_of_laps import SlamData


def odomCallback(odomMsg: Odometry, slamData: SlamData) -> None:
    """
    Callback function to be exceuted on subscribing to odometry message
    """
    positionX, positionY, yaw = slamData.parse(odomMsg)
    slamData.saveOdometry(positionX, positionY, yaw)


def conesCallback(conesMsg: LandmarkArray, slamData: SlamData) -> None:
    """
    Callback function to be exceuted on subscribing to landmark array message
    landmarks are stored in an array which is afterwards used to
    perform the process of clustring the cones
    """
    cones = parseLandmarks(conesMsg)[:, :3]
    slamData.addCones(cones)


def main() -> None:
    """
    the main running program which includes the definition of
    all the publishers and subscribers

    Subscribers:
    the node subscribes to:
    1- odometry message from kiss_icp
    2- LandmarkArray from perceprtion node


    Publishers:
    the node publishes:
    1- Number of completed laps (roadstate message)
    2- clustered cones (landmarkArray)

    """
    slamData = SlamData()
    rospy.init_node("Slam_node")
    # global slamData
    markerViz = MarkerViz(0.2, 0.4)

    rospy.Subscriber("/odometry_node/odometry", Odometry, odomCallback, callback_args=slamData)

    roadstatePub = rospy.Publisher("/SLAM/road_state", Roadstate, queue_size=1)

    rospy.Subscriber(
        "/perception/smornn/detected", LandmarkArray, conesCallback, callback_args=slamData
    )

    landmarkPub = rospy.Publisher("/cones_map", LandmarkArray, queue_size=1)

    markerPub = rospy.Publisher("/cones_map_visualization", MarkerArray, queue_size=1)

    # Main loop
    mainRate = 10
    rateR = rospy.Rate(mainRate)
    while not rospy.is_shutdown():
        rateR.sleep()
        laps, distance = slamData.lapsCompleted, slamData.distTravelled
        roadState = Roadstate()
        roadState.header.stamp = rospy.Time.now()
        roadState.laps = laps
        roadState.distance = distance
        roadstatePub.publish(roadState)

        clusteredCones, bestColors = slamData.getCones()
        landmarks = createLandmarkMessage(clusteredCones, bestColors, "odom")
        landmarkPub.publish(landmarks)
        conesVisual = markerViz.conesToMarkers(landmarks)
        markerPub.publish(conesVisual)


if __name__ == "__main__":
    main()
