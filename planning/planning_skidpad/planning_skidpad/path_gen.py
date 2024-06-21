"""Module providing a function printing python version."""
# import time
import rclpy
from rclpy.node import Node
from kinematic_bicycle.SkidPadPathPlanningClass import SendPath
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from asurt_msgs.msg import LandmarkArray


class SkidPad(Node):  # type: ignore[misc]
    """
    Class for generating a path for the skidpad.

    Attributes:
        pathGen (SendPath): The path generation object.
        finalPath (Path): The final path to be published.
        state (Odometry): The current state of the vehicle.
        conePositions (np.array): The positions of the cones.

    """

    def __init__(self) -> None:
        super().__init__("ConesPublisher")

        # Publishers
        self.pathPub = self.create_publisher(Path, "/path", 10)

        # Subscriptions
        self.create_subscription(LandmarkArray, "cones", self.listenerCallback, 10)
        self.create_subscription(Odometry, "state", self.stateCallback, 10)

        # Initialize variables
        self.pathGen = SendPath()
        self.finalPath = Path()
        self.state = Odometry()
        self.conePositions = np.empty((0, 3))
        # self.timeStart = 0.0
        # self.timeStep = 0.0

    def stateCallback(self, state: Odometry) -> None:
        """
        Callback function for handling state updates.

        Args:
            state (Odometry): The new state information.

        Returns:
            None
        """
        # Update the current state
        self.state = state

    def listenerCallback(self, poseArrayMsg: LandmarkArray) -> None:
        """
        Callback function for handling PoseArray messages.

        Args:
            poseArrayMsg (PoseArray): The incoming PoseArray message.

        Returns:
            None
        """
        # Extract cone positions from the PoseArray message
        self.conePositions = np.append(
            self.conePositions,
            np.array(
                [
                    [pose.position.x, pose.position.y, pose.identifier]
                    for pose in poseArrayMsg.landmarks
                ]
            ),
            axis=0,
        )

        # Generate the path
        self.finalPath.poses = []
        # self.timeStart = time.time()
        path, self.conePositions = self.pathGen.getPath(self.state, self.conePositions)
        # self.timeStep = time.time() - self.timeStart
        self.get_logger().info(f"time_step is {self.timeStep} ")

        if path is None:
            return

        # Convert the path to PoseStamped messages and publish it
        for i in path:
            pose = PoseStamped()
            pose.pose.position.x = float(i[0])
            pose.pose.position.y = float(i[1])
            self.finalPath.poses.append(pose)
        self.finalPath.header.frame_id = "map"
        self.pathPub.publish(self.finalPath)


def main() -> None:
    """
    Main function that initializes the program and executes the path generation.

    Args:
        args: Command-line arguments.

    Returns:
        None
    """
    # Initialize the ROS2 node
    rclpy.init()
    # Create the path generation node
    path = SkidPad()
    # Spin the node
    rclpy.spin(path)
    # Destroy the node
    path.destroy_node()
    # Shutdown the ROS2 client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
