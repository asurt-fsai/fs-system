from ..helpers.intervalTimer import IntervalTimer
from  ..helpers.missionLauncher import *

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int16
from tf_helper.StatusPublisher import StatusPublisher

import threading
from eufs_msgs.msg import CanState




class TestModule(Node):

    def __init__(self) -> None:
        super().__init__("testnode")
        self.launcher = MissionLauncher()

    def run(self):
        self.launcher.launch(CanState.AMI_DDT_INSPECTION_A)

def main():
    rclpy.init()
    test=TestModule()
    test.run()
    rclpy.spin(test)



if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:
        pass