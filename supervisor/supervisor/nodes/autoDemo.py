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
        self.started = False
        self.status = StatusPublisher("/status/autonomous_demo", self)

        self.declare_parameter('maxSteer',rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("/control/velocity", rclpy.Parameter.Type.STRING)
        self.declare_parameter("/control/steering", rclpy.Parameter.Type.STRING)
        self.declare_parameter('/state', rclpy.Parameter.Type.STRING)
        self.declare_parameter('/finisher/is_finished', rclpy.Parameter.Type.STRING)
        self.declare_parameter("/vcu/curr_vel", rclpy.Parameter.Type.STRING)
        self.declare_parameter('/ros_can/ebs', rclpy.Parameter.Type.STRING) 
        self.declare_parameter("/supervisor/driving_flag", rclpy.Parameter.Type.STRING)

        
    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True
            #self.run()
         


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
        ebsTopic = self.get_parameter('/ros_can/ebs').get_parameter_value().string_value
        self.get_logger().info(ebsTopic)

        velPub = self.create_publisher(Float32, velTopic, 10)
        steerPub = self.create_publisher(Float32, steerTopic, 10)
        statePub = self.create_publisher(Int16, stateTopic, 10)
        finishPub = self.create_publisher(Bool, isFinishedTopic, 10)
        distPub = self.create_publisher(Float32, '/distance', 10)


        maxSteer = Float32()
        maxSteer = maxSteerDouble
        #maxSteer.data = 27.2
    
        time.sleep(2)
        statePub.publish(Int16(data=2))  # 2 is for driving
        steerPub.publish(Float32(data=-maxSteer))
        time.sleep(3)
        steerPub.publish(Float32(data=maxSteer))
        time.sleep(3)
        steerPub.publish(Float32(data=0.0))
        time.sleep(3)

        timeStart = time.time()
        distance = 0
        initial_velocity = 0  
        acceleration = 1  
        deceleration = -1

        while distance < 10: # 10m is the target distance
            currentTime = time.time()
            timeElapsed = currentTime - timeStart 

            velocity = initial_velocity + acceleration * timeElapsed
            vel = Float32(data=velocity)
            velPub.publish(vel)
            
            distance = initial_velocity * timeElapsed + 0.5 * acceleration * (timeElapsed**2)
            dist = Float32(data=distance)
            distPub.publish(dist)
            time.sleep(0.01)

        self.get_logger().info("distance: " + str(distance))
        self.get_logger().info("vel: " + str(velocity * 3.6))
        time.sleep(3)

        distance = 10
        timeStart = time.time()
        initial_velocity = velocity 
        while  distance < 20 and velocity > 0:
            currentTime = time.time()
            timeElapsed = currentTime - timeStart 

            velocity = initial_velocity + deceleration * timeElapsed
            vel = Float32(data=velocity)
            velPub.publish(vel)
            
            distance = 10 + velocity * timeElapsed + 0.5 * abs(deceleration) * (timeElapsed**2)
            dist = Float32(data=distance)
            distPub.publish(dist)

            time.sleep(0.01)

        velPub.publish(Float32(data= 0.0))
        self.get_logger().info("distance: " + str(distance))
        self.get_logger().info("vel: " + str(velocity * 3.6))
        time.sleep(2)


        timeStart = time.time()
        initial_velocity = 0
        while distance < 30:
            currentTime = time.time()
            timeElapsed = currentTime - timeStart 

            velocity = initial_velocity + acceleration * timeElapsed
            vel = Float32(data=velocity)
            velPub.publish(vel)
            
            distance = 20+ initial_velocity * timeElapsed + 0.5 * acceleration * (timeElapsed**2)
            dist = Float32(data=distance)
            distPub.publish(dist)
            time.sleep(0.01)
            if distance >= 30 : break

        self.get_logger().info("finished " )


        ###########################
        #CALLING EBS SERVICE HERE
        ###########################
        
        client=self.create_client(Trigger, ebsTopic)  # define the client
        self.get_logger().info("1")
        while not client.wait_for_service(timeout_sec=1.0): # wait for the server to be up
            self.get_logger().warn('waiting for server')
            # define the request to be called
        self.get_logger().info("2")
        request = Trigger.Request()  # Define the request inside the loop
        self.get_logger().info("3")

        # call async is non-blocking meaning that it will not wait for the response.
        future_obj = client.call_async(request)
        self.get_logger().info(str(future_obj))
        self.get_logger().info("4")
        rclpy.spin_until_future_complete(self, future_obj) # spin until the response object is received
        self.get_logger().info("5")
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
    Autonomous Demo, publish stuff
    """

    rclpy.init()
    autonomousDemo = AutonomousDemo()
    drivingFlagTopic = autonomousDemo.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    autonomousDemo.create_subscription(Bool, drivingFlagTopic, autonomousDemo.drivingFlagCallback, 10)
    autonomousDemo.create_service(Trigger,"/ros_can/ebs",autonomousDemo.callback)
    autonomousDemo.status.starting()
    autonomousDemo.status.ready()
    autonomousDemo.get_logger().info("1")
    rate = autonomousDemo.create_rate(10)
    while rclpy.ok():
        autonomousDemo.status.running()
        autonomousDemo.get_logger().info("2")
        if autonomousDemo.started:
            autonomousDemo.get_logger().info("3")
            autonomousDemo.run()
            break

        rclpy.spin_once(autonomousDemo, timeout_sec=0.1)
        autonomousDemo.get_logger().info("4")
        #rclpy.spin(autonomousDemo) #the spin must exist to enter the callback for the service

        #rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
