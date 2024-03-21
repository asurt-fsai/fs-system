#!/usr/bin/python3
"""
.

"""
import rclpy
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from supervisor.helpers.rosCanSimulator import RosCanSimulator


def main() -> None:
    """
    Main function.
    """
    
    rclpy.init()
    node = rclpy.create_node("ros_can_simulator")
    
    drivingFlagTopic = node.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    missionFlagTopic = node.get_parameter("/supervisor/mission_flag").get_parameter_value().string_value
    rosCanCmdTopic = node.get_parameter("/ros_can/cmd").get_parameter_value().string_value
    vcuCurrVelTopic = node.get_parameter("/vcu/curr_vel").get_parameter_value().string_value
    vcuVelTopic = node.get_parameter("/vcu/control_vel").get_parameter_value().string_value
    vcuSteerTopic = node.get_parameter("/vcu/control_steer").get_parameter_value().string_value
    rosCanStateTopic = node.get_parameter("/ros_can/can_state").get_parameter_value().string_value
    rosCanVelTopic = node.get_parameter("/ros_can/twist").get_parameter_value().string_value


    rosCanSimulator = RosCanSimulator(
        18, vcuVelTopic, vcuSteerTopic, rosCanStateTopic, rosCanVelTopic
    )
    
    node.create_subscription(Bool, drivingFlagTopic, rosCanSimulator.drivingFlagCallback, 10)
    node.create_subscription(Bool, missionFlagTopic, rosCanSimulator.missionFlagCallback, 10)
    node.create_subscription(AckermannDriveStamped, rosCanCmdTopic, rosCanSimulator.cmdCallback, 10)
    node.create_subscription(Float32, vcuCurrVelTopic, rosCanSimulator.currentVelCallback, 10)

    rate = node.create_rate(100)
    
    try:
        while rclpy.ok():
            rosCanSimulator.run()
            rate.sleep()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()