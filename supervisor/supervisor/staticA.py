#!/usr/bin/python3
"""
Static A
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
#from tf_helper.StatusPublisher import StatusPublisher
from .intervalTimer import IntervalTimer


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
        velTopic = "/velocity"#self.get_parameter("/control/velocity").get_parameter_value()
        steerTopic = "/steer"#self.get_parameter("/control/steering").get_parameter_value()
        isFinishedTopic = "/isFinished" #self.get_parameter("/finisher/is_finished").get_parameter_value()
        stateTopic = "/state" #self.get_parameter("/state").get_parameter_value()


        velPub = self.create_publisher(Float32, velTopic, 10)
        steerPub = self.create_publisher(Float32, steerTopic, 10)
        statePub = self.create_publisher(Int16, stateTopic, 10)
        #finishPub = self.create_publisher(Bool, isFinishedTopic, 10, latch=True)
        maxSteer = Float32()#float(self.get_parameter("/maxSteer").get_parameter_value())
        #maxSteer.data =float(self.get_parameter("/maxSteer").get_parameter_value()) #30.0
        maxSteer.data = 30.0
        
        velocity = Float32()
        velocity.data = 10.0
        velPub.publish(velocity)
        time.sleep(2)
        statePub.publish(Int16(data=1))  # 1 is for driving
        steerPub.publish(Float32(data=-maxSteer.data))
        time.sleep(10)
        steerPub.publish(Float32(data=maxSteer.data))
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

  #  heartbeartRateThread = IntervalTimer(0.5, status.running)

    #drivingFlagTopic = staticA.get_parameter("/supervisor/driving_flag").get_parameter_value().string_value
    #StaticA.create_subscription(Bool, drivingFlagTopic, staticA.drivingFlagCallback, 10)

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



