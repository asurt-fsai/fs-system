"""
this module is the controller node for the kinematic bicycle model.
The controller node subscribes to the state and path topics
The controller node publishes the steering angle and throttle to the respective topics.
It uses the adaptive pure pursuit algorithm to calculate the steering angle and throttle.
It also uses a PID controller to calculate the throttle based on the steering angle.

The controller node is implemented as a class with the following methods:

- initPubAndSub: Initializes the publishers and subscribers for the controller node.

- stateCallback: Callback function for the state subscriber.

- pathCallback: Callback function for the path subscriber.

- publishDrive: Publishes the drive message to the drive topic.

- main: Main function to initialize the controller node.

"""

import math
from typing import List
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from .adaptive_purepursuit import AdaptivePurePursuit


class Controller(Node):  # type: ignore[misc]
    """
    Controller class for the kinematic bicycle model.

    args:
        Node: rclpy node object

    functions:
        __init__: Initializes the controller node.

        declareTopics: Declares the topics for the controller node
        as parameters in the parameter server.

        initPubAndSub: Initializes the publishers and subscribers for the controller node.

        stateCallback: Callback function for the state subscriber.

        pathCallback: Callback function for the path subscriber.

        publishDrive: Publishes the drive message to the drive topic.

        main: Main function to initialize the controller node.

    """

    def __init__(self) -> None:
        """
        Initializes the controller node with the name "controller" using the super() function.
        """
        super().__init__("Controller")
        self.purepursuit = AdaptivePurePursuit(self)
        self.declareTopics()
        self.initPubAndSub()

    def declareTopics(self) -> None:
        """
        Declare the topics for the controller node as parameters in the parameter server.

        topics:
            drive: Topic to publish the drive message
            state: Topic to subscribe to the state message
            path: Topic to subscribe to the path message
        """
        self.declare_parameter("drive", rclpy.Parameter.Type.STRING)
        self.declare_parameter("state", rclpy.Parameter.Type.STRING)
        self.declare_parameter("path", rclpy.Parameter.Type.STRING)
        self.get_logger().info("parameters declared")

    def initPubAndSub(self) -> None:
        """
        Initializes the publishers and subscribers for the controller node.

        topics:
            drive_topic: Topic to publish the drive message
            state_topic: Topic to subscribe to the state message
            path_topic: Topic to subscribe to the path message

        publishers:
            drivePub: Publisher for the drive message

        subscribers:
            stateSub: Subscriber for the state message
            pathSub: Subscriber for the path message

        timer:
            timer: Timer to publish the drive message
        """
        driveTopic = self.get_parameter("drive").get_parameter_value().string_value
        log = "drive topic : " + str(driveTopic)
        stateTopic = self.get_parameter("state").get_parameter_value().string_value
        log = log + "state topic : " + str(stateTopic)
        pathTopic = self.get_parameter("path").get_parameter_value().string_value
        log = log + "path topic : " + str(pathTopic)
        self.get_logger().info(log)

        self.drivePub = self.create_publisher(AckermannDriveStamped, driveTopic, 10)
        self.stateSub = self.create_subscription(Odometry, stateTopic, self.stateCallback, 10)
        self.pathSub = self.create_subscription(Path, pathTopic, self.pathCallback, 10)
        self.timer = self.create_timer(0.1, self.publishDrive)

    def stateCallback(self, state: Odometry) -> None:
        """
        Callback function for the state subscriber.

        Args:
            state: Odometry message

        state:
            x: x position of the vehicle from the odometry message
            y: y position of the vehicle from the odometry message
            yaw: yaw angle of the vehicle from the odometry message
            velocity: velocity of the vehicle from the odometry message
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

        path:
            waypoints: List of waypoints from the path message
        """
        self.purepursuit.waypoints = [
            (pose.pose.position.x, pose.pose.position.y) for pose in path.poses
        ]

    def publishDrive(self) -> None:
        """
        Publishes the drive message to the drinodeve topic.

        driveMsg:
            AckermannDriveStamped message
            drive.steering_angle: steering angle calculated by the adaptive pure pursuit algorithm
            drive.speed: current vehicle's velocity plus throttle calculated by the PID controller

        steeringAngle:
            steering angle calculated by the adaptive pure pursuit algorithm

        throttle:
            throttle calculated by the PID controller
        """
        driveMsg = AckermannDriveStamped()
        if len(self.purepursuit.waypoints) > 0:
            steeringAngle = self.purepursuit.angleCalc()
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
