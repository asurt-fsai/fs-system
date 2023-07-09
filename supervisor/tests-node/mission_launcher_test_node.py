#!/usr/bin/python3
"""
.

"""
import rospy
from supervisor.helpers import MissionLauncher  # type: ignore[attr-defined]


def main() -> None:
    """
    Main function.
    """
    rospy.init_node("mission_launcher_test_node")
    missionLauncher = MissionLauncher("/visualizer/text", "/visualizer/buttons")
    isBagSystem = rospy.get_param("/supervisor/is_bag_system")
    missionLauncher.launch(14, isBagSystem)  # placeholder for test mission

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        missionLauncher.update()
        rate.sleep()
    missionLauncher.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
