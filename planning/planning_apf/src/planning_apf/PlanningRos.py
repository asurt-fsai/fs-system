"""
This module contains the PlanningRos class which is used to subscribe to cone 
positions and colors, and publish a planned path.
"""

from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from asurt_msgs.msg import LandmarkArray, Landmark
from geometry_msgs.msg import PoseStamped
import numpy as np
import numpy.typing as npt
import rospy
import time
from .APF import APF

prevTime = 0
prevTimeCPU = 0


class PlanningRos:  # pylint: disable=too-many-instance-attributes
    """
    Initializes the PlanningRos class.

    Parameters
    ----------
    conesTopic : str
        The topic to subscribe to for cone positions.
    pubTopic : str
        The topic to publish the planned path to.
    tfHelper : tfHelper
        The tfHelper object to transform cone positions to the world frame.
    frameId : str
        The frame id of the world frame.
    kAttractive : float
        The attractive force constant.
    kRepulsive : float
        The repulsive force constant.
    repulsiveRadius : float
        The repulsiveRadius constant.
    stepSize : float
        The step size.
    maxIterations : int
        The maximum number of iterations.
    goalThreshold : float
        The threshold distance from the goal.
    isIpg : bool
        True if the cones are from IPG, false otherwise.
    plot : bool, optional
        True if the path should be plotted, false otherwise.

    parameters
    ----------
    pub : rospy.Publisher
        The publisher to publish the planned path to.

    tfHelper : tfHelper
        The tfHelper object to transform cone positions to the world frame.
    frameId : str
        The frame id of the world frame.
    kAttractive : float
        The attractive force constant.
    kRepulsive : float
        The repulsive force constant.
    repulsiveRadius : float
        The repulsiveRadius constant.
    stepSize : float
        The step size.
    maxIterations : int
        The maximum number of iterations.
    goalThreshold : float
        The threshold distance from the goal.
    blueCones : list
        A list of blue cones in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    yellowCones : list
        A list of yellow cones in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    allCones : list
        A list of all cones in the form of [(x1, y1), (x2, y2), ..., (xn, yn)].
    plot : bool, optional
        True if the path should be plotted, false otherwise.

    """

    def __init__(
        self,
        conesTopic: str,
        pubTopic: str,
        tfHelper,
        frameId: str,
        kAttractive: float,
        kRepulsive: float,
        repulsiveRadius: float,
        stepSize: float,
        maxIterations: int,
        goalThreshold: float,
        isIpg: bool,
        plot: bool = False,
    ):  # pylint: disable=too-many-arguments
        self.plot = plot
        self.pub = rospy.Publisher(pubTopic, Path, queue_size=10)
        if isIpg:
            rospy.Subscriber(conesTopic, MarkerArray, self.markerCallback, queue_size=10)
        else:
            rospy.Subscriber(conesTopic, LandmarkArray, self.perceptionCallback, queue_size=10)
        self.tfHelper = tfHelper
        self.frameId = frameId
        self.kAttractive = kAttractive
        self.kRepulsive = kRepulsive
        self.repulsiveRadius = repulsiveRadius
        self.stepSize = stepSize
        self.maxIterations = maxIterations
        self.goalThreshold = goalThreshold
        self.blueCones = []
        self.yellowCones = []
        self.allCones = []

    def numpyToPath(self, pathArray: npt.NDArray[np.float64]) -> Path:
        """
        Converts a numpy array to a path message
        and transforms the coordinates to the world frame
        Parameters
        ----------
        pathArray: np.ndarray, shape=(n, 2)
            Numpy array with x and y coordinates of the path
        Returns
        -------
        path: Path
            Path message
        """
        achievedPath = Path()
        for i in range(pathArray.shape[0]):
            pose = PoseStamped()
            pose.pose.position.x = pathArray[i][0]
            pose.pose.position.y = pathArray[i][1]
            pose.header.frame_id = self.frameId
            pose.header.stamp = rospy.Time.now()
            achievedPath.poses.append(pose)
        achievedPath.header.frame_id = self.frameId
        return achievedPath

    def markerCallback(self, msg: MarkerArray) -> None:
        """
        Callback function to process marker messages and extract cone positions.

        Parameters
        ----------
        msg : Marker
            The marker message containing cone information.

        Returns
        -------
        None.
        """
        self.yellowCones = []
        self.blueCones = []
        self.allCones = []
        for marker in msg.markers:
            red, green, blue = marker.color.r, marker.color.g, marker.color.b
            xPosition, yPosition = marker.pose.position.x, marker.pose.position.y
            if red == 1 and green == 1 and blue == 0:
                self.yellowCones.append([xPosition, yPosition])
                self.allCones.append([xPosition, yPosition])
            elif red == 0 and blue == 1:
                self.blueCones.append([xPosition, yPosition])
                self.allCones.append([xPosition, yPosition])

    def perceptionCallback(self, landmarkArray: npt.NDArray[np.float64]) -> None:
        """
        This callback function is called whenever a new message of type LandmarkArray is
        received by the subscriber.

        Parameters
        ----------
        LandmarkArray : LandmarkArray
            The message received by the subscriber.

        Returns
        -------
        None.

        """
        landmarkArray = self.tfHelper.transformMsg(landmarkArray, self.frameId)
        self.yellowCones = []
        self.blueCones = []
        self.allCones = []
        for landmark in landmarkArray.landmarks:
            if landmark.type == Landmark.BLUE_CONE:
                self.blueCones.append(np.array([landmark.position.x, landmark.position.y]))
                self.allCones.append(np.array([landmark.position.x, landmark.position.y]))
            elif landmark.type == Landmark.YELLOW_CONE:
                self.yellowCones.append(np.array([landmark.position.x, landmark.position.y]))
                self.allCones.append(np.array([landmark.position.x, landmark.position.y]))
            elif landmark.type in [
                Landmark.CONE_TYPE_UNKOWN,
                Landmark.ORANGE_CONE,
                Landmark.LARGE_CONE,
            ]:
                self.allCones.append(np.array([landmark.position.x, landmark.position.y]))
                print(self.allCones)
                print("---------------------")

    def getConesLength(self):
        return len(self.allCones)

    def run(self, counter):
        global prevTime, prevTimeCPU
        """
        Runs the path planning algorithm and publishes the planned path.

        Returns:
        path.
        """
        print(len(self.allCones))
        if len(self.allCones) > 0:
            apfTest = APF(
                (0, 0),
                (0, 0),
                self.allCones,
                self.kAttractive,
                self.kRepulsive,
                self.repulsiveRadius,
                self.stepSize,
                self.maxIterations,
                self.goalThreshold,
                self.yellowCones,
                self.blueCones,
                self.plot,
            )
            start = time.time()
            startCPU = time.process_time()
            apfTest.pathPlanPlot()
            end = time.time()
            endCPU = time.process_time()
            # execution Time calculation
            print("Time: ", end - start)
            print("CPU Time: ", endCPU - startCPU)
            totalTime = end - start
            totalTimeCPU = endCPU - startCPU
            prevTime += totalTime
            prevTimeCPU += totalTimeCPU
            average = prevTime / counter
            averageCPU = prevTimeCPU / counter
            print("Average: ", average)
            print("Average CPU: ", averageCPU)
            path = Path()
            npath = np.array(apfTest.path)
            path = self.numpyToPath(npath)
            self.pub.publish(path)
            return path
