from eufs_msgs.msg import WheelSpeedsStamped
from std_msgs.msg import Header
import rclpy
from rclpy.node import Node
import time

def make_wheel_speeds_message():
    msg = WheelSpeedsStamped()
    msg.header = Header()
    msg.header.stamp = rclpy.time.Time().to_msg()
    msg.header.frame_id = 'base_footprint'

    msg.speeds.lf_speed = 20.0
    msg.speeds.rf_speed = 20.0
    msg.speeds.lb_speed = 5.0
    msg.speeds.rb_speed = 20.0
    msg.speeds.steering = 10.0

    return msg

class WheelSpeedsPublisher(Node):

    def __init__(self):
        super().__init__('vehicle_command_publisher')
        self.publisher = self.create_publisher(WheelSpeedsStamped, '/ros_can/wheel_speeds', 10)
    
        msg = make_wheel_speeds_message()
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedsPublisher()
    #rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
