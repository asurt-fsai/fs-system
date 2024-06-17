#!/usr/bin/python3
"""
Static B
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
import time
from tf_helper.StatusPublisher import StatusPublisher
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
        super().__init__('StaticB_Node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('/control/velocity', rclpy.Parameter.Type.STRING),
                ('/ros_can/ebs', rclpy.Parameter.Type.STRING),
                ('/finisher/is_finished', rclpy.Parameter.Type.STRING),
                ('/state', rclpy.Parameter.Type.STRING),
                ('/supervisor/driving_flag', rclpy.Parameter.Type.STRING),
                
                
                        
            ])
        self.drivingFlag = False
        self.started = False
        self.status = StatusPublisher("/status/staticB", self)

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.get_logger().info("inside driving callback")
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.get_logger().info("inside if condition")
            self.started = True
            #self.run()

    #@staticmethod
    def run(self) -> None:
        """
        Run Static B
        """
        self.get_logger().info("Starting Static B")
        
        velTopic = self.get_parameter("/control/velocity").get_parameter_value().string_value
        isFinishedTopic = self.get_parameter("/finisher/is_finished").get_parameter_value().string_value
        stateTopic = self.get_parameter("/state").get_parameter_value().string_value
        ebsTopic = self.get_parameter("/ros_can/ebs").get_parameter_value().string_value

        velPub = self.create_publisher(Float32, velTopic, 10)
        statePub = self.create_publisher(Int16, stateTopic, 10)
        finishPub = self.create_publisher(Bool, isFinishedTopic, 10)


        time.sleep(10)
        
        vel = Float32(data =(2 * np.pi * 50 * 0.253 / 60))  # 50 rpm
        statePub.publish(Int16(data=2))
        self.get_logger().info("yes")
        velPub.publish(vel)
        self.get_logger().info(str(vel))
        time.sleep(10)

        ############################

        # CALLING EBS SERVICE HERE

        ############################

        self.get_logger().warn('ebs called')
        client=self.create_client(Trigger,ebsTopic)#define the client
        while not client.wait_for_service(timeout_sec=0.25): # wait for the server to be up
            self.get_logger().warn('waiting for server')
            #define the request to be called
        request=Trigger.Request()
        
        #call async is non blocking mean that it will not wait for the response.
        future_obj = client.call_async(request) 
        rclpy.spin_until_future_complete(self, future_obj) # spin until the response object is received
        self.get_logger().warn('DOOOOOOOOOONE')
        

        statePub.publish(Int16(data=3))  # 3 is for EBS
        msg = Bool()
        msg.data = True
        finishPub.publish(msg) 

    def callback(self,request:Trigger.Request,response:Trigger.Response):
        response.success=True
        response.message = "hey there"
        self.get_logger().info("sending back response from logger:"+ str(response.success))
        print("sending back response:",response.success)
        return response
    

def main() -> None:
    """
    Static B, publish stuff
    """

    rclpy.init()
    staticB = StaticB()
    staticB.status.starting()
    staticB.status.ready()

    #heartbeartRateThread = IntervalTimer(0.5, staticB.status.running)
    drivingFlagTopic = staticB.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    staticB.create_subscription(Bool, drivingFlagTopic, staticB.drivingFlagCallback, 10)


    rate = staticB.create_rate(10)
    
    #heartbeartRateThread.start()
    while rclpy.ok():
        staticB.status.running()
        staticB.get_logger().info("inside while")
        staticB.create_service(Trigger, "/ros_can/ebs",staticB.callback)
        if staticB.started:
            
            staticB.get_logger().info("runn")
            staticB.run()
            break
        
        rclpy.spin_once(staticB, timeout_sec=0.1)
        

    #rclpy.spin(staticB)    

        # while rclpy.ok():
        #     staticB.get_logger().info("inside second while")
        #     rclpy.spin(staticB) #the spin must exist to enter the callback for the service
        # pass



    
    rclpy.shutdown() 


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
