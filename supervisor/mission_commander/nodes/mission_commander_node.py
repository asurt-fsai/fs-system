"""
mission_commander module for the supervisor package.
"""
import rospy

from std_msgs.msg import Bool

from tf_helper.StatusPublisher import StatusPublisher
from mission_commander.helpers.intervalTimer import IntervalTimer
from mission_commander.staticA import StaticA
from mission_commander.staticB import StaticB
from mission_commander.autonomous_demo import AutonomousDemo
from mission_commander.autocross import Autocross
from mission_commander.track_drive import TrackDrive


def main() -> None:
    """
    Static A, publish stuff
    """

    rospy.init_node("mission_commander")

    velTopic = rospy.get_param("/control/velocity")
    steerTopic = rospy.get_param("/control/steering")
    isFinishedTopic = rospy.get_param("/finisher/is_finished")
    ebsTopic = rospy.get_param("/ros_can/ebs")
    stateTopic = rospy.get_param("/state")
    missionType = rospy.get_param("/mission_type")
    drivingFlagTopic = rospy.get_param("/supervisor/driving_flag")

    currentMission = None
    statusPublisherThread = None

    status = StatusPublisher(f"/status/{missionType}")
    status.starting()
    status.ready()

    if missionType == "staticA":
        currentMission = StaticA(velTopic, steerTopic, isFinishedTopic, stateTopic)
        statusPublisherThread = IntervalTimer(0.5, status.running)
    elif missionType == "staticB":
        currentMission = StaticB(velTopic, ebsTopic, isFinishedTopic, stateTopic)
        statusPublisherThread = IntervalTimer(0.5, status.running)
    elif missionType == "autonomousDemo":
        currentMission = AutonomousDemo(velTopic, steerTopic, ebsTopic, isFinishedTopic, stateTopic)
    elif missionType == "autocross":
        currentMission = Autocross(isFinishedTopic)
    elif missionType == "trackDrive":
        currentMission = TrackDrive(isFinishedTopic)

    else:
        currentMission = StaticA(velTopic, steerTopic, isFinishedTopic, stateTopic)

    rospy.Subscriber(drivingFlagTopic, Bool, currentMission.drivingFlagCallback)

    rate = rospy.Rate(10)

    if statusPublisherThread:
        statusPublisherThread.start()

    while not rospy.is_shutdown():
        if not statusPublisherThread:
            status.running()
        currentMission.run()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
