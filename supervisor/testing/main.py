# pylint: disable=all
# mypy: ignore-errors

import rospy
from supervisor.helpers import MissionLauncher

if __name__ == "__main__":
    rospy.init_node("mission_launcher")
    launcher = MissionLauncher(None)
    launcher.launch(-1)
    rospy.spin()
