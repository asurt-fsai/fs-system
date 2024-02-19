#!/usr/bin/python3
"""
Static B
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
import time
#from tf_helper.StatusPublisher import StatusPublisher
import numpy as np
from std_srvs.srv import Trigger


class StaticB(Node):
    """
    Static B
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(self) -> None:
        super().__init__('staticB_node')
        self.drivingFlag = False
        self.started = True

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True

    #@staticmethod
    def run(self) -> None:
        """
        Run Static B
        """
        self.get_logger().info("Starting Static B")

        velTopic = "/velocity"
        isFinishedTopic = "/isFinished" 
        stateTopic = "/state" 
        ebsTopic = "/ros_can/ebs"


       #velTopic = self.get_parameter("/control/velocity").get_parameter_value().string_value
        #isFinishedTopic = self.get_parameter("/finisher/is_finished").get_parameter_value().string_value
        #stateTopic = self.get_parameter("/state").get_parameter_value().string_value
        #ebsTopic = self.get_parameter("/ros_can/ebs").get_parameter_value().string_value

        velPub = self.create_publisher(Float32, velTopic, 10)
        statePub = self.create_publisher(Int16, stateTopic, 10)
        finishPub = self.create_publisher(Bool, isFinishedTopic, 10)


        time.sleep(2)
        vel = 2 * np.pi * 50 * 0.253 / 60  # 50 rpm
        velPub.publish(Float32(data=vel))
        time.sleep(10)

        ############################

        # CALLING EBS SERVICE HERE

        ############################
        
           
        # Create the connection to the service. Remember it's a Trigger service
      # Create the connection to the service
        self.ebs_service = self.create_client(Trigger, ebsTopic)

        # Wait for the service to be available
        # while not self.ebs_service.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
        self.ebs_service.wait_for_service()
        # Create an object of the type Trigger::Request
        ebs_request = Trigger.Request()

        # Now send the request through the connection
        future = self.ebs_service.call_async(ebs_request)
        future.add_done_callback(self.callback)


        statePub.publish(2)  # 2 is for finished
        # finishPub.publish(True)  # 1 is for finished

    def callback( self,future):
        try:
            response = future.result()
            self.get_logger().info('Service call was successful with response: %r' % response)
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main() -> None:
    """
    Static B, publish stuff
    """

    rclpy.init()
    staticB = StaticB()
    #drivingFlagTopic = staticB.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    #staticB.create_subscription(Bool, drivingFlagTopic, staticB.drivingFlagCallback, 10)

    #status = StatusPublisher("/status/staticB")
    #status.starting()
    #status.ready()

    rate = staticB.create_rate(10)
    while rclpy.ok():
        #status.running()
        if staticB.started:
            staticB.run()
            break

        rclpy.spin(staticB)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
