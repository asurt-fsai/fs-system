#!/usr/bin/python3
"""
.

"""
import rospy
from std_msgs.msg import Float32, Int16, Bool
from supervisor.supervisor_functions.missionLauncher import MissionLauncher
from supervisor.supervisor_functions.republisher import Republisher


def main() -> None:
    """
    Main function.
    """

    rospy.init_node("supervisor")

    repub = Republisher()
    missionType = MissionLauncher()
    rospy.Subscriber("/vel", Float32, repub.velCallback)
    rospy.Subscriber("/steer", Float32, repub.steerCallback)
    rospy.Subscriber("/mission_type", Int16, missionType.missionCallback)
    rospy.Subscriber("/go_station", Bool, missionType.goCallback)
    rospy.Subscriber("/state", Int16, missionType.stateCallback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        missionType.setVel(repub.vel)
        missionType.setSteer(repub.steer)
        missionType.launch()
        rate.sleep()

    # Pool.
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
