from eufs_msgs.msg import VehicleCommandsStamped
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
import time

def make_vehicle_commands_message():
    msg = VehicleCommandsStamped()
    msg.header = Header()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.header.frame_id = 'base_footprint'

    # Populate msg.commands fields directly
    msg.commands.ebs = 0  # Example value
    msg.commands.braking = 0.5  # Example value
    msg.commands.torque = 0.0  # Example value
    msg.commands.steering = 0.0  # Example value
    msg.commands.rpm = 3000.0  # Example value
    msg.commands.handshake = 1  # Example value
    msg.commands.direction = 0  # Example value
    msg.commands.mission_status = 2  # Example value

    return msg

class VehicleCommandPublisher(Node):

    def __init__(self):
        super().__init__('vehicle_command_publisher')
        self.publisher = self.create_publisher(VehicleCommandsStamped, '/ros_can/vehicle_commands', 10)
    
        msg = make_vehicle_commands_message()
        
        
        msg.commands.direction = 0 
        msg.commands.mission_status = 3  
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleCommandPublisher()
    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
