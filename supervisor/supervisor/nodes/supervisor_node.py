#!/usr/bin/python3
"""
.

"""
import rclpy
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState
from geometry_msgs.msg import TwistWithCovarianceStamped
# from tf_helper.src.tf_helper.StatusPublisher import StatusPublisher
from .supervisor import Supervisor  # type: ignore



def main() -> None:
    """
    Main function.
    """

    rclpy.init()
    node = rclpy.create_node("supervisor")

    node.declare_parameter("/control/velocity", rclpy.Parameter.Type.STRING)
    node.declare_parameter("/ros_can/cmd", rclpy.Parameter.Type.STRING)
    node.declare_parameter('/supervisor/driving_flag', rclpy.Parameter.Type.STRING)
    node.declare_parameter('/supervisor/mission_flag', rclpy.Parameter.Type.STRING)
    node.declare_parameter('/finisher/is_finished', rclpy.Parameter.Type.STRING)
    node.declare_parameter('/ros_can/can_state', rclpy.Parameter.Type.STRING)
    node.declare_parameter('/ros_can/twist', rclpy.Parameter.Type.STRING)
    node.declare_parameter('/control/steering', rclpy.Parameter.Type.STRING)
    node.declare_parameter('/visualizer/marker', rclpy.Parameter.Type.STRING)
    node.declare_parameter('/visualizer/button', rclpy.Parameter.Type.STRING) 

    #status = StatusPublisher("/status/supervisor")
    #status.starting()

    controlVelTopic = node.get_parameter("/control/velocity").get_parameter_value().string_value

    rosCanCmdTopic = node.get_parameter("/ros_can/cmd").get_parameter_value().string_value
    drivingFlagTopic = node.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    missionFlagTopic = node.get_parameter("/supervisor/mission_flag").get_parameter_value().string_value
    isFinishedTopic = node.get_parameter("/finisher/is_finished").get_parameter_value().string_value
    rosCanStateTopic = node.get_parameter("/ros_can/can_state").get_parameter_value().string_value
    rosCanVelTopic = node.get_parameter("/ros_can/twist").get_parameter_value().string_value
    controlSteerTopic = node.get_parameter("/control/steering").get_parameter_value().string_value

    markerTopic = node.get_parameter("/visualizer/marker").get_parameter_value().string_value
    btnTopic = node.get_parameter("/visualizer/button").get_parameter_value().string_value


    supervisor = Supervisor(
       rosCanCmdTopic, drivingFlagTopic, missionFlagTopic, markerTopic, btnTopic
    )
    

    node.create_subscription(CanState, rosCanStateTopic, supervisor.canStateCallback, 10)
    node.create_subscription(Bool, isFinishedTopic, supervisor.isFinishedCallback, 10)
    node.create_subscription(TwistWithCovarianceStamped, rosCanVelTopic, supervisor.currentVelCallback, 10)
    node.create_subscription(Float32, controlVelTopic, supervisor.velCallback, 10)
    node.create_subscription(Float32, controlSteerTopic, supervisor.steerCallback, 10)

    
    rate = node.create_rate(100)
    
    try:
        while rclpy.ok():
            supervisor.run()
            rate.sleep()

            #status.running()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
