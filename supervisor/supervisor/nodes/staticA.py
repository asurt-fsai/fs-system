#!/usr/bin/python3
"""
Static A
"""
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
#from tf_helper.src.tf_helper.StatusPublisher import StatusPublisher
from ..helpers.intervalTimer import IntervalTimer


#from sleep import sleepForSecondsAndSendHeartBeat
import numpy as np


class StaticA(Node):
    """
    Static A
    -----------------------
    Attributes:
        started: bool
    -----------------------
    returns:
        None
    """

    def __init__(self) -> None:
        super().__init__("StaticA_Node")
        self.declare_parameter('maxSteer',rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter("/control/velocity", rclpy.Parameter.Type.STRING)
        self.declare_parameter("/control/steering", rclpy.Parameter.Type.STRING)
        self.declare_parameter("/finisher/is_finished", rclpy.Parameter.Type.STRING)
        self.declare_parameter('/state', rclpy.Parameter.Type.STRING)
        

        self.drivingFlag = False
        self.started = True

    def drivingFlagCallback(self, msg: Bool) -> None:
        """
        Callback for the driving flag
        """
        self.drivingFlag = msg.data
        if not self.started and self.drivingFlag:
            self.started = True

    def run(self) -> None:
        """
        Run Static A
        """
        self.get_logger().info("Starting Static A")
        velTopic = self.get_parameter("/control/velocity").get_parameter_value().string_value
        steerTopic = self.get_parameter("/control/steering").get_parameter_value().string_value
        isFinishedTopic =  self.get_parameter("/finisher/is_finished").get_parameter_value().string_value
        stateTopic = self.get_parameter("/state").get_parameter_value().string_value


        velPub = self.create_publisher(Float32, velTopic, 10)
        steerPub = self.create_publisher(Float32, steerTopic, 10)
        statePub = self.create_publisher(Int16, stateTopic, 10)
        #finishPub = self.create_publisher(Bool, isFinishedTopic, 10, latch=True)

        
        maxSteerDouble =self.get_parameter("maxSteer").get_parameter_value().double_value
        print("MAX STEER ISSS!!!!!" , end= '')
        print(maxSteerDouble)
        
        maxSteer = Float32()
        maxSteer = maxSteerDouble
        print(maxSteer)
        
        
        velocity = Float32()
        velocity.data = 10.0
        velPub.publish(velocity)
        time.sleep(2)
        statePub.publish(Int16(data=1))  # 1 is for driving
        steerPub.publish(Float32(data=-maxSteer))
        time.sleep(10)
        steerPub.publish(Float32(data=maxSteer))
        time.sleep(10)
        steerPub.publish(Float32(data=0.0))
        time.sleep(10)

        timeStart = time.time()
        while time.time() - timeStart < 10:
            
            vel =Float32(data=(2 * np.pi * 200 * 0.253 / 60 * (time.time() - timeStart)))
            velPub.publish(vel)
            time.sleep(0.1)


        time.sleep(2)
        velPub.publish(Float32(data=0.0))
        time.sleep(2)

        statePub.publish(Int16(data=2))  # 2 is for finished
        #finishPub.publish(True)  # 1 is for finished
        
    



def main() -> None:
    """
    Static A, publish stuff
    """

    rclpy.init()
    
   # status = StatusPublisher("/status/staticA")
    staticA = StaticA()
    
    #status.starting()
   # status.ready()

    #heartbeartRateThread = IntervalTimer(0.5, status.running)

    # drivingFlagTopic = staticA.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    # StaticA.create_subscription(Bool, drivingFlagTopic, staticA.drivingFlagCallback, 10)

    rate = staticA.create_rate(10)
    #heartbeartRateThread.run()

    while rclpy.ok():
        if staticA.started:
            staticA.run()
            break

        rate.sleep()

    rclpy.shutdown()    


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass



