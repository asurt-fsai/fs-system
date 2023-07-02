"""
indication for the interpreter that should be used to run the script.
"""
# pylint: disable=too-many-instance-attributes
# pylint: disable=global-variable-not-assigned
# pylint: disable=global-statement

import functools
from typing import Callable, List, Any
import typing
from threading import Lock
import rospy
from nav_msgs.msg import Odometry
from asurt_msgs.msg import Roadstate, LandmarkArray
from tf_helper.MarkerViz import MarkerViz
from tf_helper.utils import parseLandmarks, createLandmarkMessage
from tf_helper.TFHelper import TFHelper
from visualization_msgs.msg import MarkerArray
import numpy as np
from numpy.typing import NDArray
from trapper.srv import ResetAttributes


# from tf_helper import *
from tf.transformations import euler_from_quaternion


def mutexLock(lock: Lock) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """
    Decorator to lock a function with a given mutex

    Parameters
    ----------
    lock : threading.Lock
        Mutex lock to fetch and release
    """

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        @functools.wraps(func)
        def newFunc(*args: Any, **kwargs: Any) -> Any:
            lock.acquire()
            val = None
            try:
                val = func(*args, **kwargs)
            finally:
                lock.release()
            return val

        return newFunc

    return decorator


class SlamData:
    """
    SlamData is a class which has the attributes required for
    1- Clustring the cones such as: minimum number of times the cone has been detected,
       the minumim distance between 2 cones to be considered separate

    2- Determining number of laps completed and distance travel as it has attributes
       that stores the values of first position,last position and distance travel.
       Moreover, it has an attribute used to determine if the current position is far from the
       first position

    it has 5 Methods saveOdometry,parse,addCones,clusteredCones,get_cones

    """

    mutex = Lock()

    def __init__(self) -> None:
        # self.didClosure = True
        # self.gtPositions: NDArray[Any] = np.array([])
        self.clusteredCone: List[float] = []
        self.unclusteredCones: List[List["float"]] = []
        self.framesCones: List[List[List[float]]] = []
        self.clusteredColors: List[List["int"]] = []
        self.clusteredconesCount: List["int"] = []
        # self.conePoseids: NDArray[Any] = np.array([])
        # self.pointclouds: NDArray[Any] = np.array([])
        self.lastPosition = np.zeros(3)
        # self.counter = 0
        # self.lastAddedpose = None
        self.distTravelled: float = 0
        self.lapsCompleted = 0
        self.firstPosition = [0, 0, 0]
        self.isfarFromfirst = False
        self.callbackCount = 0
        self.tfHelper = TFHelper("trapper")
        # self.minOccurcount = 3
        # self.coneRaduis = 1

    def resetAttributes(self, req: Any) -> None:  # pylint: disable=unused-argument
        """
        ResetAttributes service callback function
        """
        self.clusteredCone = []
        self.unclusteredCones = []
        self.framesCones = []
        self.clusteredColors = []
        self.clusteredconesCount = []
        self.lastPosition = np.zeros(3)
        self.distTravelled = 0
        self.lapsCompleted = 0
        self.firstPosition = [0, 0, 0]
        self.isfarFromfirst = False
        self.callbackCount = 0

    def run(self) -> None:
        """
        The run method is used to initialize the node and the service
        """
        serviceName = "reset"
        rospy.Service(serviceName, ResetAttributes, self.resetAttributes)

    def saveOdometry(self, positionX: "float", positionY: "float", yaw: "float") -> None:
        """
        1-saves the first odometry message array in first position attribute to have a reference for
          distance measurnment and lap detection by determing if the current pose was moving away
          from the first position and then approaching it

        2-saves odometry message arrays in last position attribute iteritably
          to compute the needed calculations

        """
        if self.firstPosition is None:
            self.firstPosition = np.array([positionX, positionY, yaw])
        else:
            dist = np.linalg.norm(self.lastPosition[:2] - np.array([positionX, positionY])).astype(
                float
            )
            self.distTravelled += dist
            self.lastPosition = np.array([positionX, positionY, yaw])
            lastPose = self.lastPosition
            if self.firstPosition is not None:
                distTofirst = np.linalg.norm(lastPose[:2] - self.firstPosition[:2])
                if distTofirst > 10:
                    self.isfarFromfirst = True

                if self.isfarFromfirst and distTofirst < 3:
                    self.isfarFromfirst = False
                    self.lapsCompleted += 1

    def parse(self, odomMsg: Odometry) -> typing.Tuple[float, float, float]:
        """
        this method parses the incoming odometry message to get
        quaternion info then tranforms it to euler

        """
        position = odomMsg.pose.pose.position
        quat = odomMsg.pose.pose.orientation
        positionX = quat.x
        positionY = quat.y
        positionZ = quat.z
        positionW = quat.w

        euler = euler_from_quaternion((positionX, positionY, positionZ, positionW))
        return position.x, position.y, euler[2]  # (x, y, yaw)

    def addCones(self, parsedCones: NDArray[Any]) -> None:
        """
        Transforms the given cones from the robot's local coordinate system
        to the global coordinate system,
        using the robot's last known position and orientation,
        and adds them to the list of unclustered cones.

        Args:
            parsedCones: A list of cones, where each cone is represented as a list with 3 elements:
                         - The X position of the cone in the robot's local coordinate system.
                         - The Y position of the cone in the robot's local coordinate system.
                         - The type of the cone.

        Returns:
            None.
        """
        poseX, poseY, yaw = self.lastPosition
        rotMat = np.eye(3)
        rotMat[0][0] = np.cos(yaw)
        rotMat[0][1] = -1 * np.sin(yaw)
        rotMat[1][0] = np.sin(yaw)
        rotMat[1][1] = np.cos(yaw)

        transformedCones = []
        for cone in parsedCones:
            conex, coneY, coneType = cone

            # Transform point
            point = np.array([conex, coneY, 1]).reshape(3, 1)
            transformed = rotMat @ point
            conex = transformed[0][0] + poseX
            coneY = transformed[1][0] + poseY

            transformedCones.append([conex, coneY, coneType])

        self.framesCones.append(transformedCones)
        self.framesCones = self.framesCones[-10:]
        self.unclusteredCones = []
        for cones in self.framesCones:
            self.unclusteredCones.extend(cones)

    @mutexLock(mutex)
    def clusteredCones(self) -> None:
        """
        Method Workflow:
        1. Check if there are any unclustered cones available for processing.
        2. If unclustered cones exist, create a new numpy array
        and copy the unclustered cones into it.
        3. Empty the `unclusteredCones` list.
        4. Create an empty numpy array `clusteredCones` to store clustered cones.
        5. If `clusteredCones` list already contains elements, convert it into a numpy array and
          keep only the x and y coordinates of the cones.
        6. Loop through each new cone in the `newCones` numpy array.
        7. Calculate the distance between the new cone
        and all the cones in the `clusteredCones` list.
        8. If a nearby cone exists, merge the new cone with the nearest cone
        by averaging their x and y coordinates
        and updating the cone type counts in the `clusteredColors` list.
        9. Otherwise, append the new cone to the `clusteredCones` list
        and initialize the cone type counts in the `clusteredColors` list.
        10. Convert `clusteredCones` list back to numpy array
        and keep only the x and y coordinates of the cones.

        """

        if len(self.unclusteredCones) > 0:
            newCones = np.copy(np.array(self.unclusteredCones)).astype(float)
            self.unclusteredCones = []
            clusteredCones: NDArray[Any] = np.array([])
            if len(self.clusteredCone) > 0:
                clusteredCones = np.array(self.clusteredCone)[:, :2].astype(float)

            for newCone in newCones:
                minDist = 9999
                minIdx: np.intp = np.intp(0)
                if clusteredCones.shape[0] > 0:
                    dists = np.linalg.norm(
                        np.subtract(newCone[:2].reshape(1, 2), clusteredCones), axis=1
                    )
                    minIdx = np.argmin(dists)
                    minDist = dists[minIdx]

                if minDist < 1:
                    coneCount = self.clusteredconesCount[minIdx]
                    self.clusteredCone[minIdx] = (
                        clusteredCones[minIdx] * coneCount + newCone[:2]
                    ) / (coneCount + 1)
                    self.clusteredColors[minIdx][int(newCone[2])] += 1
                    clusteredCones = np.array(self.clusteredCones)[:, :2].astype(float)  # Very slow
                    self.clusteredconesCount[minIdx] += 1
                else:
                    self.clusteredCone.append(newCone[:2])
                    self.clusteredconesCount.append(1)
                    self.clusteredColors.append([0, 0, 0, 0, 0])
                    self.clusteredColors[minIdx][int(newCone[2])] += 1
                    clusteredCones = np.array(self.clusteredCone)[:, :2].astype(float)  # Very slow

    def getCones(self) -> typing.Tuple[NDArray[Any], NDArray[Any]]:
        """
        The getCones method takes the clustered cones and their colors,
        filters them based on a threshold, and returns the filtered cones and their colors.
        """
        self.clusteredCones()
        if len(self.clusteredCone) == 0:
            return np.array([]), np.array([])

        counts = np.array(self.clusteredconesCount)
        clusteredCones = np.array(self.clusteredCone)[:, :2].astype(float)[counts > 3]
        clusteredColors = np.array(self.clusteredColors)[counts > 3]
        probsColors: List["float"] = []
        for clusterC in clusteredColors:
            probsColors.append(clusterC[:3] / (np.sum(clusterC[:3] + 1)))

        if len(probsColors) == 0:
            bestColors: NDArray[Any] = np.array([])
        else:
            bestColors = np.argmax(probsColors, axis=1)
            bestColors[np.max(probsColors, axis=1) < 0.4] = 4

        return clusteredCones, bestColors

    @mutexLock(mutex)
    def odomCallback(self, odomMsg: Odometry) -> None:
        """
        Callback function to be exceuted on subscribing to odometry message
        """
        reqFrame = rospy.get_param("toID")
        transformedOdom = self.tfHelper.transformMsg(odomMsg, reqFrame)
        positionX, positionY, yaw = self.parse(transformedOdom)
        self.saveOdometry(positionX, positionY, yaw)

    @mutexLock(mutex)
    def conesCallback(self, conesMsg: LandmarkArray) -> None:
        """
        Callback function to be exceuted on subscribing to landmark array message
        landmarks are stored in an array which is afterwards used to
        Perform the process of clustring the cones
        """
        self.callbackCount += 1
        reqFrame = rospy.get_param("toID")
        transformedCones = self.tfHelper.transformMsg(conesMsg, reqFrame)
        cones = parseLandmarks(transformedCones)
        if len(cones) > 0:
            self.addCones(cones[:, :3])


class RosTrapper(SlamData):
    """
    roswrapper for SlamData class
    """

    def __init__(self) -> None:
        super().__init__()
        odometryTopic = rospy.get_param("Odometry_Topic")
        roadstateTopic = rospy.get_param("Roadstate_Topic")
        landmarkTopic = rospy.get_param("Landmarks_Topic")
        mapTopic = rospy.get_param("Map_Topic")
        visualizationTopic = rospy.get_param("Map_Viz_Topic")
        self.frameId = rospy.get_param("toID")
        self.odomSub = rospy.Subscriber(odometryTopic, Odometry, self.odomCallback)
        self.conesSub = rospy.Subscriber(landmarkTopic, LandmarkArray, self.conesCallback)
        self.roadstatePub = rospy.Publisher(roadstateTopic, Roadstate, queue_size=1)
        self.landmarkPub = rospy.Publisher(mapTopic, LandmarkArray, queue_size=1)
        self.markerPub = rospy.Publisher(visualizationTopic, MarkerArray, queue_size=1)
        self.markerViz = MarkerViz(0.2, 0.4)

    def runClustercones(self) -> None:
        """
        publishes the clustered cones and the number of completed laps
        """
        clusteredCones, bestColors = self.getCones()
        landmarks = createLandmarkMessage(
            clusteredCones, bestColors, np.ones(bestColors.shape), self.frameId
        )
        self.landmarkPub.publish(landmarks)
        conesVisual = self.markerViz.conesToMarkers(landmarks)
        self.markerPub.publish(conesVisual)

    def publishSupervisordata(self) -> None:
        """
        Method Workflow:
        publish the number of completed laps and the distance travelled
        """
        laps, distance = self.lapsCompleted, self.distTravelled
        roadState = Roadstate()
        roadState.header.stamp = rospy.Time.now()
        roadState.laps = laps
        roadState.distance = distance
        self.roadstatePub.publish(roadState)
