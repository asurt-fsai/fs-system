# from supervisor import GUI

# GUI.gui

from std_msgs.msg import Bool
from eufs_msgs.msg import CanState
from rclpy.node import Node
import rclpy
import time

class testing(Node):
    def __init__(self):
        super().__init__("testttt")
        self.create_subscription(CanState, "/ros_can/state", self.callback, 10)
        pub =self.create_publisher(Bool, "shutDownCircuit",1) 
        time.sleep(5)
        x = Bool()
        x.data = True
        pub.publish(x)



    def callback(self, msg):
        self.get_logger().info(str(msg))


def main():
    rclpy.init()
    test=testing()
    rclpy.spin(test)

    


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        pass            


