"""
A module to fit to lines for left and right cones and publish way points for control to follow
"""
from typing import Union, Dict, List, Any
import rospy
import numpy as np
from asurt_msgs.msg import LandmarkArray
from tf_helper.utils import parseLandmarks
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import LinearRegression
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class AccPlanner:
    """
    A class to fit to lines for left and right cones and publish way points
    for control to follow in acceleration dynamic envent
    """

    def __init__(
        self,
        trackWidth: float,
        lengthOfPath: Union[float, int],
        pathResolution: float,
        filterRange: float,
        topics: Dict[str, str],
    ) -> None:
        self.params: Dict[str, float] = {
            "trackWidth": trackWidth,
            "lengthOfPath": lengthOfPath,
            "pathResolution": pathResolution,
            "filterRange": filterRange,
        }
        self.topics: Dict[str, str] = topics
        self.leftCones: List[Any] = []
        self.rightCones: List[Any] = []
        self.centerLine = LinearRegression()
        self.centerLine.coef_ = np.array([0])
        self.centerLine.intercept_ = np.array([0])
        self.pathPub: rospy.Publisher
        self.conesPub: rospy.Publisher

    def createPublishers(self) -> None:
        """
        Method to create publishers for path and cones
        """
        self.pathPub = rospy.Publisher(self.topics["path"], Path, queue_size=10)
        self.conesPub = rospy.Publisher(self.topics["cones"], MarkerArray, queue_size=10)

    def mapCallback(self, msg: LandmarkArray) -> None:
        """
        a callback function to get cones of the current map from (Trapper Module)

        It simply parses the cones from the message and filters them
        based on the range of the center line and then splits them into
        left and right cones based on their position relative to the center line

        Parameters
        ----------
        msg : LandmarkArray
            a message that contains the cones of the current map
        """
        cones = parseLandmarks(msg)
        # plot cones
        # Filter cones not within the range of the center line
        dists = [
            abs(cone[1] - self.centerLine.predict(np.array([cone[0]]).reshape(-1, 1)))
            for cone in cones
        ]
        cones = [cone for cone, dist in zip(cones, dists) if dist < self.params["filterRange"]]

        # Split cones into left and right based on the center line
        self.leftCones = [
            cone
            for cone in cones
            if cone[1] < self.centerLine.predict(np.array([cone[0]]).reshape(-1, 1))
        ]
        self.rightCones = [
            cone
            for cone in cones
            if cone[1] > self.centerLine.predict(np.array([cone[0]]).reshape(-1, 1))
        ]

    def getCenterLine(self) -> None:
        """
        A method to get the center line of the track from the left and right cones

        if there are more than 2 cones on both sides, it uses RANSAC to
        fit a line to the cones for both sides and then averages the parameters
        of the two lines to get the center line.

        if there is only one cone on one side, it assumes a perpindicular line
        that goes through the cone and then shifts it over by half the track width


        """
        if len(self.leftCones) >= 2 and len(self.rightCones) >= 2:
            leftX = np.array([cone[0] for cone in self.leftCones]).reshape(-1, 1)
            leftY = np.array([cone[1] for cone in self.leftCones]).reshape(-1, 1)
            rightX = np.array([cone[0] for cone in self.rightCones]).reshape(-1, 1)
            rightY = np.array([cone[1] for cone in self.rightCones]).reshape(-1, 1)
            leftModel = RANSACRegressor()
            leftModel = leftModel.fit(leftX, leftY)
            rightModel = RANSACRegressor()
            rightModel = rightModel.fit(rightX, rightY)

            # Get center line by averging parameters between left and right
            leftParams = leftModel.estimator_.coef_[0][0], leftModel.estimator_.intercept_[0]
            rightParams = rightModel.estimator_.coef_[0][0], rightModel.estimator_.intercept_[0]
            avgParams = np.mean([leftParams, rightParams], axis=0)

            self.centerLine = LinearRegression()
            self.centerLine.coef_ = np.array([avgParams[0]])
            self.centerLine.intercept_ = np.array([avgParams[1]])
        else:
            if len(self.leftCones) == 1 and len(self.rightCones) == 1:
                # Assume perpindicular line that goes through the midpoint of the two cones
                # Now the center line is the same line shifted over by half the track width
                self.centerLine = LinearRegression()
                self.centerLine.coef_ = np.array([0])
                self.centerLine.intercept_ = np.array(
                    [(self.leftCones[0][0] + self.rightCones[0][0]) / 2]
                )
            if len(self.leftCones) >= 1:
                # Assume perpindicular left line that goes through the left cone
                # Now the center line is the same line shifted over by half the track width
                self.centerLine = LinearRegression()
                self.centerLine.coef_ = np.array([0])
                self.centerLine.intercept_ = np.array(
                    [self.leftCones[0][1] + self.params["trackWidth"] / 2]
                )
            elif len(self.rightCones) >= 1:
                # Assume perpindicular right line that goes through the right cone
                # Now the center line is the same line shifted over by half the track width
                self.centerLine = LinearRegression()
                self.centerLine.coef_ = np.array([0])
                self.centerLine.intercept_ = np.array(
                    [self.rightCones[0][1] - self.params["trackWidth"] / 2]
                )

    def plan(self) -> Path:
        """
        Method to plan a path for the car to follow
        by generating waypoints in front of the car
        that follow the center line obtained from the map
        """
        self.getCenterLine()

        # Generate waypoints
        odom = rospy.wait_for_message(self.topics["odom"], Odometry)
        curPose = odom.pose.pose
        curPos = np.array([curPose.position.x, curPose.position.y])

        # publish path message for points in front of the car
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        x = curPos[0]
        while x < curPos[0] + self.params["lengthOfPath"]:
            x += self.params["pathResolution"]
            y = self.centerLine.predict(np.array([x]).reshape(-1, 1))
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)

        self.pathPub.publish(path)

        self.publishCones()

        return path

    def publishCones(self) -> None:
        """
        Publishes the cones as markers to rviz
        """
        markerArray = MarkerArray()
        markerArray.markers = []

        idx = 0
        for cone in self.rightCones:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.id = idx
            marker.action = marker.ADD
            marker.pose.position.x = cone[0]
            marker.pose.position.y = cone[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.b = 1.0
            markerArray.markers.append(marker)
            idx += 1
        # Right cones with color yellow
        for cone in self.leftCones:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = idx
            marker.pose.position.x = cone[0]
            marker.pose.position.y = cone[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            markerArray.markers.append(marker)
            idx += 1
        self.conesPub.publish(markerArray)

    def start(self) -> None:
        """
        Method to start the planner
        """
        rospy.Subscriber(self.topics["map"], LandmarkArray, self.mapCallback)
        self.createPublishers()
