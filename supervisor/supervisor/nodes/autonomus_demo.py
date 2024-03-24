#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
from std_srvs.srv import Trigger
from tf_helper.StatusPublisher import StatusPublisher

class AutonomousDemo(Node):
    """
    Autonomous Demo
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(self) -> None:
        super().__init__("Auto_demo_node")
        self.drivingFlag = False
        self.started = True
        self.currentVel = 0.0
        self.timePrevStep = time.time()
        self.dist = 0.0
        self.drevT = 0.0
        self.status = StatusPublisher("/status/autonomous_demo", self)

        self.declare_parameter('maxSteer',rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("/control/velocity", rclpy.Parameter.Type.STRING)
        self.declare_parameter("/control/steering", rclpy.Parameter.Type.STRING)
        self.declare_parameter('/state', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/finisher/is_finished', rclpy.Parameter.Type.STRING)
        self.declare_parameter("/vcu/curr_vel", rclpy.Parameter.Type.STRING)
        self.declare_parameter("/ros_can/ebs", rclpy.Parameter.Type.STRING)
        self.declare_parameter("/supervisor/driving_flag", rclpy.Parameter.Type.STRING)


        
        
    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True

    def currentVelCallback(self, msg: Float32) -> None:
        """
        Callback function
        """
        timeCurrentStep = time.time()
        self.drevT = timeCurrentStep - self.timePrevStep
        self.timePrevStep = timeCurrentStep
        self.currentVel = msg.data

    def run(self) -> None:
        """
        Run Autonomous Demo
        """
        self.get_logger().info("Starting Autonomous Demo")

        
        velTopic = self.get_parameter("/control/velocity").get_parameter_value().string_value
        isFinishedTopic = self.get_parameter("/finisher/is_finished").get_parameter_value().string_value
        stateTopic = self.get_parameter("/state").get_parameter_value().string_value
        steerTopic = self.get_parameter("/control/steering").get_parameter_value().string_value
        vcuCurrVelTopic = self.get_parameter("/vcu/curr_vel").get_parameter_value().string_value
        maxSteerDouble = self.get_parameter("maxSteer").get_parameter_value().double_value
        ebsTopic = float(self.get_parameter("/ros_can/ebs").get_parameter_value().double_value)

        self.create_subscription(Float32, vcuCurrVelTopic, self.currentVelCallback, 10)
        velPub = self.create_publisher(Float32, velTopic, 10)
        steerPub = self.create_publisher(Float32, steerTopic, 10)
        statePub = self.create_publisher(Int16, stateTopic, 10)
        finishPub = self.create_publisher(Int16, isFinishedTopic, 10)

        

        maxSteer = Float32()
        maxSteer = maxSteerDouble
        #maxSteer.data = 27.2
    
        time.sleep(2)
        statePub.publish(Int16(data=1))  # 1 is for driving
        steerPub.publish(Float32(data=-maxSteer))
        time.sleep(10)
        steerPub.publish(Float32(data=maxSteer))
        time.sleep(10)
        steerPub.publish(Float32(data=0.0))
        time.sleep(10)

        while self.dist < 10:
            self.dist += self.currentVel * self.drevT
            velPub.publish(Float32(data=self.dist))

        time.sleep(5)
        velPub.publish(Float32(data=0.0))
        time.sleep(5)

        self.dist = 0.0
        while self.dist < 10:
            self.dist += self.currentVel * self.drevT
            velPub.publish(Float32(data=self.dist))


        ############################
        # CALLING EBS SERVICE HERE
        ############################
        client=self.create_client(Trigger,ebsTopic)#define the client
        while not client.wait_for_service(timeout_sec=0.25): # wait for the server to be up
            self.get_logger().warn('waiting for server')
            #define the request to be called
        request=Trigger.Request()
        
        #call async is non blocking mean that it will not wait for the response.
        future_obj = client.call_async(request) 
        rclpy.spin_until_future_complete(self, future_obj) # spin until the response object is received
        self.get_logger().warn('DOOOOOOOOOONE')
        ############################
    
        statePub.publish(Int16(data=2))  # 2 is for finished
        finishPub.publish(Int16(data=1))  # 1 is for finished

def callback(request:Trigger.Request,response:Trigger.Response):
    response.success=True
    response.message = "hey there"
    print("sending back response:",response.success)
    return response

def main() -> None:
    """
    Autonomous Demo, publish stuff
    """

    rclpy.init()
    autonomousDemo = AutonomousDemo()
    drivingFlagTopic = autonomousDemo.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    #drivingFlagTopic = "/supervisor/driving_flag"
    autonomousDemo.create_subscription(Bool, drivingFlagTopic, autonomousDemo.drivingFlagCallback, 10)
    
    autonomousDemo.status.starting()
    autonomousDemo.status.ready()

    rate = autonomousDemo.create_rate(10)
    while rclpy.ok():
        autonomousDemo.status.running()
        autonomousDemo.create_service(Trigger, "/ros_can/ebs",callback)
        if autonomousDemo.started:
            autonomousDemo.run()
            break

        while rclpy.ok():
            rclpy.spin(autonomousDemo) #the spin must exist to enter the callback for the service
        pass


        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
