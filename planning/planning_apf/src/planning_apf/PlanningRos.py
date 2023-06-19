from nav_msgs.msg import Path
import rospy
from visualization_msgs.msg import MarkerArray
from asurt_msgs.msg import LandmarkArray, Landmark
from .APF import APF
import numpy as np
import numpy.typing as npt
from geometry_msgs.msg import PoseStamped


class PlanningRos:
    def __init__(
        self,
        cones_topic: str,
        pub_topic: str,
        tf_helper,
        frame_id: str,
        is_ipg: bool = False,
        plot: bool = False,
    ):
        self.plot = plot
        self.pub = rospy.Publisher(pub_topic, Path, queue_size=10)
        if is_ipg:
            rospy.Subscriber(cones_topic, MarkerArray, self.markerCallback, queue_size=10)
        else:
            rospy.Subscriber(cones_topic, LandmarkArray, self.perceptionCallback, queue_size=10)
        self.tf_helper = tf_helper
        self.frame_id = frame_id

        self.blue_cones = []
        self.yellow_cones = []
        self.all_cones = []

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
            pose.header.frame_id = self.frame_id
            pose.header.stamp = rospy.Time.now()
            achievedPath.poses.append(pose)
        achievedPath.header.frame_id = self.frame_id
        return achievedPath

    def markerCallback(self, msg):
        self.yellow_cones = []
        self.blue_cones = []
        self.all_cones = []
        for marker in msg.markers:
            r, g, b = marker.color.r, marker.color.g, marker.color.b
            x, y = marker.pose.position.x, marker.pose.position.y
            if r == 1 and g == 1 and b == 0:
                self.yellow_cones.append([x, y])
                self.all_cones.append([x, y])
            elif r == 0 and b == 1:
                self.blue_cones.append([x, y])
                self.all_cones.append([x, y])

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
        landmarkArray = self.tf_helper.transformMsg(landmarkArray, self.frame_id)
        self.yellow_cones = []
        self.blue_cones = []
        self.all_cones = []
        for landmark in landmarkArray.landmarks:
            if landmark.type == Landmark.BLUE_CONE:
                self.blue_cones.append(np.array([landmark.position.x, landmark.position.y]))
                self.all_cones.append(np.array([landmark.position.x, landmark.position.y]))
            elif landmark.type == Landmark.YELLOW_CONE:
                self.yellow_cones.append(np.array([landmark.position.x, landmark.position.y]))
                self.all_cones.append(np.array([landmark.position.x, landmark.position.y]))
            elif landmark.type in [
                Landmark.CONE_TYPE_UNKOWN,
                Landmark.ORANGE_CONE,
                Landmark.LARGE_CONE,
            ]:
                self.all_cones.append(np.array([landmark.position.x, landmark.position.y]))

    def run(self):
        print(len(self.all_cones))
        if len(self.all_cones) > 0:
            apfTest = APF(
                (0, 0),
                (0, 0),
                self.all_cones,
                3.5,
                30,
                0.9,
                0.2,
                25,
                0.2,
                self.yellow_cones,
                self.blue_cones,
                self.plot,
            )
            apfTest.pathPlanPlot()

            path = Path()
            npath = np.array(apfTest.path)
            path = self.numpyToPath(npath)
            self.pub.publish(path)
