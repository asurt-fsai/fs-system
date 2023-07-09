"""
SwitchingNode used to switch between aloam and hdl
"""
import rospy
import rosnode
import tf2_ros

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from asurt_msgs.msg import Roadstate
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from .mapMerger import MapMerger


class SwitchingNode:  # pylint: disable=too-many-instance-attributes, too-many-arguments
    """
    SwitchingNode used to switch between aloam and hdl

    Parameters
    ----------
    lidarTopic:str
        Topic from velodyne_pointcloud driver
    slamLidarTopic:str
        Topic to publish pointclouds for slam on
    roadStateTopic: str
        Topic from the trapper, used for counting when the first lap is finished
    aloamOdomTopic: str
        Topic for odometry from aloam
    hdlOdomTopic: str
        Topic for odometry from hdl
    initPoseTopic: str
        Topic for requesting to publish current pose
    odomPubTopic: str
        Topic where odometry is published
    loadMapTopic: str
        Topic to request loading the map from hdl
    initPosePubTopic: str
        Topic where the initPose request poses are published to
    dataDir: str
        Path to the pointclouds saved by aloam
    minPcCountToStart: int
        Minimum number of pointclouds to start sending pointclouds to slam
    """

    def __init__(
        self,
        lidarTopic: str,
        slamLidarTopic: str,
        roadStateTopic: str,
        aloamOdomTopic: str,
        hdlOdomTopic: str,
        initPoseTopic: str,
        odomPubTopic: str,
        loadMapTopic: str,
        initPosePubTopic: str,
        dataDir: str,
        minPcCountToStart: int,
    ):
        rospy.init_node("switching_node")

        self.tfBroadcaster = tf2_ros.StaticTransformBroadcaster()

        self.odomPub = rospy.Publisher(odomPubTopic, Odometry, queue_size=1)
        self.loadMapRequest = rospy.Publisher(loadMapTopic, Bool, queue_size=1)
        self.initPosePub = rospy.Publisher(
            initPosePubTopic, PoseWithCovarianceStamped, queue_size=1
        )
        self.pcPub = rospy.Publisher(slamLidarTopic, PointCloud2, queue_size=10)

        self.mapMerger = MapMerger(dataDir)
        self.lap = 0

        self.latestOdom = Odometry()
        self.latestOdom.header.frame_id = "map"
        self.latestOdom.pose.pose.orientation.w = 1
        self.isHdlStarted = False
        self.isMapMerged = False
        self.pcCount = 0
        self.minPcCountToStart = minPcCountToStart

        self.roadstateSub = rospy.Subscriber(roadStateTopic, Roadstate, self.roadstateCallback)
        self.aloamSub = rospy.Subscriber(aloamOdomTopic, Odometry, self.aloamCallback)
        rospy.Subscriber(hdlOdomTopic, Odometry, self.hdlCallback)
        rospy.Subscriber(initPoseTopic, Bool, self.publishInitPose)
        rospy.Subscriber(lidarTopic, PointCloud2, self.pcCallback)

    def pcCallback(self, pointcloud: PointCloud2) -> None:
        """
        Waits for a min number of pointcloud to have been published to start sending to slam
        """
        if self.pcCount > self.minPcCountToStart:
            self.pcPub.publish(pointcloud)
        else:
            self.pcCount += 1

    def checkLapCount(self) -> None:
        """
        Checks if a switched from aloam to hdl is needed
        If it is, the switch is performed
        """
        if self.lap == 0:
            self.loadMapRequest.publish(False)
        elif self.lap == 1 and not self.isMapMerged:
            self.isMapMerged = True
            self.mapMerger.mergeMap()
            self.loadMapRequest.publish(True)  # run hdl

    def roadstateCallback(self, data: Roadstate) -> None:
        """
        RoadState callback to fetch the number of completed laps
        """
        self.lap = data.laps
        self.checkLapCount()

    def aloamCallback(self, data: Odometry) -> None:
        """
        aloam callback, publishes odom if hdl hasn't started
        """

        if not self.isHdlStarted:
            self.latestOdom = data
        else:
            self.aloamSub.unregister()
            self.roadstateSub.unregister()
            rosnode.kill_nodes(
                [
                    "/aloam/alaserMapping",
                    "/aloam/alaserOdomerty",
                    "/aloam/alaserPGO",
                    "/aloam/ascanRegistration",
                ]
            )

    def hdlCallback(self, data: Odometry) -> None:
        """
        Hdl callback, publishes the odometry recieved from hdl
        """
        self.latestOdom = data
        self.isHdlStarted = True

    def publishInitPose(self, data: Bool) -> None:  # pylint: disable=unused-argument
        """
        Publishes the latest odometry when requested
        """
        msgToSend = PoseWithCovarianceStamped()
        msgToSend.header = self.latestOdom.header
        msgToSend.pose = self.latestOdom.pose
        self.initPosePub.publish(msgToSend)

    def broadcastTf(self) -> None:
        """
        Publishes the transform to tf according to the lastest odometry recieved
        """
        staticTfStamped = TransformStamped()

        staticTfStamped.header.stamp = rospy.Time.now()
        staticTfStamped.header.frame_id = "map"
        staticTfStamped.child_frame_id = "velodyne_fixed"

        staticTfStamped.transform.translation.x = self.latestOdom.pose.pose.position.x
        staticTfStamped.transform.translation.y = self.latestOdom.pose.pose.position.y
        staticTfStamped.transform.translation.z = self.latestOdom.pose.pose.position.z
        staticTfStamped.transform.rotation.x = self.latestOdom.pose.pose.orientation.x
        staticTfStamped.transform.rotation.y = self.latestOdom.pose.pose.orientation.y
        staticTfStamped.transform.rotation.z = self.latestOdom.pose.pose.orientation.z
        staticTfStamped.transform.rotation.w = self.latestOdom.pose.pose.orientation.w

        self.tfBroadcaster.sendTransform(staticTfStamped)

    def run(self) -> None:
        """
        Publishes the current tf and odom topics
        """
        self.odomPub.publish(self.latestOdom)
        self.broadcastTf()
