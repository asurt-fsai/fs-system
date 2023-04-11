#!/usr/bin/python3
"""
Static A
"""
import rospy
from std_msgs.msg import Float32, Bool, Int16


class StaticA:
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
        self.started = False

    def goCallback(self, msg: Bool) -> None:
        """
        Callback function
        """
        if not self.started and msg.data:
            self.started = True
            self.runStaticA()
            # Launcher

    @staticmethod
    def runStaticA() -> None:
        """
        Run Static A
        """
        rospy.loginfo("Starting Static A")
        velPub = rospy.Publisher("/vel", Float32, queue_size=10)
        steerPub = rospy.Publisher("/steer", Float32, queue_size=10)
        statePub = rospy.Publisher("/state", Int16, queue_size=10)

        rospy.sleep(2)
        statePub.publish(1)  # 1 is for driving
        steerPub.publish(-21)
        rospy.sleep(2)
        steerPub.publish(21)
        rospy.sleep(2)
        steerPub.publish(0)
        rospy.sleep(2)
        velPub.publish(10)
        rospy.sleep(2)
        velPub.publish(0)
        rospy.sleep(2)
        statePub.publish(2)  # 2 is for finished


def main() -> None:
    """
    Static A, publish stuff
    """

    mission = StaticA()

    rospy.init_node("StaticA")
    rospy.Subscriber("/go_signal", Bool, mission.goCallback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
