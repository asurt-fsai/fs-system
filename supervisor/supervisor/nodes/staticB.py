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
        self.node=super().__init__('StaticB_Node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('/control/velocity', rclpy.Parameter.Type.STRING),
                ('/ros_can/ebs', rclpy.Parameter.Type.STRING),
                ('/finisher/is_finished', rclpy.Parameter.Type.STRING),
                ('/state', rclpy.Parameter.Type.STRING),
                
                        
            ])
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

        '''
        velTopic = "/velocity"
        isFinishedTopic = "/isFinished" 
        stateTopic = "/state" 
        ebsTopic = "/ros_can/ebs"
        '''



        velTopic = self.get_parameter("/control/velocity").get_parameter_value().string_value
        isFinishedTopic = self.get_parameter("/finisher/is_finished").get_parameter_value().string_value
        stateTopic = self.get_parameter("/state").get_parameter_value().string_value
        ebsTopic = self.get_parameter("/ros_can/ebs").get_parameter_value().string_value

        velPub = self.create_publisher(Float32, velTopic, 10)
        statePub = self.create_publisher(Int16, stateTopic, 10)
        finishPub = self.create_publisher(Bool, isFinishedTopic, 10)


        time.sleep(2)
        vel = 2 * np.pi * 50 * 0.253 / 60  # 50 rpm
        velPub.publish(Float32(data=vel))
        time.sleep(10)

        ############################

        # CALLING EBS SERVICE HERE

        ############################S
    
        client=self.create_client(Trigger,ebsTopic)#define the client
        while not client.wait_for_service(timeout_sec=0.25): # wait for the server to be up
            self.get_logger().warn('waiting for server')
            #define the request to be called
        request=Trigger.Request()
        
        #call async is non blocking mean that it will not wait for the response.
        future_obj = client.call_async(request) 
        rclpy.spin_until_future_complete(self, future_obj) # spin until the response object is received
 
        
       #############################
        self.get_logger().warn('DOOOOOOOOOONE')
        # statePub.publish(2)  # 2 is for finished
        # finishPub.publish(True)  # 1 is for finished

def callback(request:Trigger.Request,response:Trigger.Response):
    response.success=True
    response.message = "hey there"
    print("sending back response:",response.success)
    return response
    

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
        staticB.create_service(Trigger, "/ros_can/ebs",callback)
        if staticB.started:
            staticB.run()
            break

        while rclpy.ok():
            rclpy.spin(staticB) #the spin must exist to enter the callback for the service
        pass



        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
