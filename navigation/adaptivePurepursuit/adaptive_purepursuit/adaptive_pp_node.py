"""nodes for adaptive pure pursuit controller"""

import math
from typing import List
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from adaptive_purepursuit.adaptive_purepursuit import AdaptivePurePursuit
from ackermann_msgs.msg import AckermannDriveStamped


class Controller(Node):  # type: ignore[misc]
    """
    Controller node for the kinematic bicycle model.

    args:
        Node: rclpy node object

    returns:
        None

    """

    def __init__(self) -> None:
        super().__init__("controller")
        self.steerPub = self.create_publisher(Float32, "steer", 10)
        self.throttlePub = self.create_publisher(Float32, "throttle", 10)
        self.drivePub = self.create_publisher(AckermannDriveStamped, "/ackr", 10)
        self.stateSub = self.create_subscription(Odometry, "/state", self.stateCallback, 10)
        self.pathSub = self.create_subscription(Path, "/path", self.pathCallback, 10)
        self.timer = self.create_timer(0.1, self.publishDrive)
        self.purepursuit = AdaptivePurePursuit(node=self)

    def initPubAndSub(self) -> None:
        """
        Initializes the publishers and subscribers for the controller node.

        args:
            None

        returns:
            None
        """
        steerTopic = self.get_parameter("control").get_parameter_value.string_value
        throttleTopic = self.get_parameter("").get_parameter_value.string_value
        driveTopic = self.get_parameter("").get_parameter_value.string_value
        stateTopic = self.get_parameter("").get_parameter_value.string_value
        pathTopic = self.get_parameter("").get_parameter_value.string_value
        self.steerPub = self.create_publisher(Float32, steerTopic, 10)
        self.throttlePub = self.create_publisher(Float32, throttleTopic, 10)
        self.drivePub = self.create_publisher(AckermannDriveStamped, driveTopic, 10)
        self.stateSub = self.create_subscription(Odometry, stateTopic, self.stateCallback, 10)
        self.pathSub = self.create_subscription(Path, pathTopic, self.pathCallback, 10)
        self.timer = self.create_timer(0.1, self.publishDrive)
        self.purepursuit = AdaptivePurePursuit(node=self)

    def stateCallback(self, state: Odometry) -> None:
        """
        Callback function for the state subscriber.

        Args:
            state: Odometry message
        """
        velocityX: float = state.twist.twist.linear.x
        velocityY: float = state.twist.twist.linear.y
        orientationList: List[float] = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w,
        ]
        self.purepursuit.state = [
            state.pose.pose.position.x,
            state.pose.pose.position.y,
            euler_from_quaternion(orientationList)[2],
            math.sqrt(velocityX**2 + velocityY**2),
        ]

    def pathCallback(self, path: Path) -> None:
        """
        Callback function for the path subscriber.pose

        Args:

            path: Path message

        Returns:
            None
        """
        self.purepursuit.waypoints = [
            (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
        ]

    def publishDrive(self) -> None:
        """
        Publishes the drive message to the drive topic.

        args:
            None

        returns:
            None
        """
        driveMsg = AckermannDriveStamped()
        if len(self.purepursuit.waypoints) > 0:
            steeringAngle = self.purepursuit.adaptivePurepursuit()
            driveMsg.drive.steering_angle = steeringAngle
            if self.purepursuit.searchTargetpoint() == len(self.purepursuit.waypoints) - 1:
                driveMsg.drive.speed = 0
            else:
                throttle = self.purepursuit.pidController(steeringAngle)
                driveMsg.drive.speed = self.purepursuit.state[3] + throttle
        else:
            driveMsg.drive.steering_angle = self.purepursuit.steeringAngle
            driveMsg.drive.speed = self.purepursuit.state[3]
        self.drivePub.publish(driveMsg)


def main() -> None:
    """main function to initialize the controller node"""
    rclpy.init()
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
