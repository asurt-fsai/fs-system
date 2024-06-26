#!/usr/bin/python3
"""
.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState
from geometry_msgs.msg import TwistWithCovarianceStamped
from tf_helper.StatusPublisher import StatusPublisher
import threading
from .supervisor import Supervisor  # type: ignore

class SupervisorNode(Node):
    def __init__(self):
        super().__init__("supervisorNode")

        self.declare_parameter("/control/velocity", rclpy.Parameter.Type.STRING)
        self.declare_parameter("/ros_can/cmd", rclpy.Parameter.Type.STRING)
        self.declare_parameter('/supervisor/driving_flag', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/supervisor/mission_flag', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/finisher/is_finished', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/ros_can/can_state', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/ros_can/twist', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/control/steering', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/visualizer/marker', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/visualizer/button', rclpy.Parameter.Type.STRING)

        self.status = StatusPublisher("/status/supervisor", self)
        self.status.starting()

        controlVelTopic = self.get_parameter("/control/velocity").get_parameter_value().string_value
        rosCanCmdTopic = self.get_parameter("/ros_can/cmd").get_parameter_value().string_value
        drivingFlagTopic = self.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
        missionFlagTopic = self.get_parameter("/supervisor/mission_flag").get_parameter_value().string_value
        isFinishedTopic = self.get_parameter("/finisher/is_finished").get_parameter_value().string_value
        rosCanStateTopic = self.get_parameter("/ros_can/can_state").get_parameter_value().string_value
        rosCanVelTopic = self.get_parameter("/ros_can/twist").get_parameter_value().string_value
        controlSteerTopic = self.get_parameter("/control/steering").get_parameter_value().string_value
        markerTopic = self.get_parameter("/visualizer/marker").get_parameter_value().string_value
        btnTopic = self.get_parameter("/visualizer/button").get_parameter_value().string_value

        self.supervisor = Supervisor(
            rosCanCmdTopic, drivingFlagTopic, missionFlagTopic, markerTopic, btnTopic
        )

        self.create_subscription(CanState, rosCanStateTopic, self.supervisor.canStateCallback, 10)
        self.create_subscription(Bool, isFinishedTopic, self.supervisor.isFinishedCallback, 10)
        self.create_subscription(TwistWithCovarianceStamped, rosCanVelTopic, self.supervisor.currentVelCallback, 10)
        self.create_subscription(Float32, controlVelTopic, self.supervisor.velCallback, 10)
        self.create_subscription(Float32, controlSteerTopic, self.supervisor.steerCallback, 10)

def main() -> None:
    """
    Main function.
    """
    rclpy.init()
    node = SupervisorNode()
    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
    rate = node.create_rate(100)

    try:
        while rclpy.ok:
            rate.sleep()
            node.supervisor.run()
            node.status.running()
            
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
