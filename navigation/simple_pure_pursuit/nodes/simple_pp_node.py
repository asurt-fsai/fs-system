"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from simple_pure_pursuit import WayPoints
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

TARGETSPEED = rospy.get_param("/speed/target", 10)


def main() -> None:
    """
    Main function for simple pure pursuit vehicle control node, subscribes to
    waypoints and publishes control actions
    """
    rospy.init_node("simple_pp_controller", anonymous=True)

    controlActionPub = rospy.Publisher("/control_actions", AckermannDriveStamped, queue_size=10)
    waypoints = WayPoints()
    targetPosePub = rospy.Publisher("/targetPoint", Marker, queue_size=10)
    targetPose = Marker()
    rospy.Subscriber("/pathplanning/waypoints", Path, callback=waypoints.add)

    controlAction = AckermannDriveStamped()

    rate = rospy.Rate(rospy.get_param("/rate", 10))
    targetInd = 0

    while not rospy.is_shutdown():

        delta, targetInd = waypoints.purepursuitSteercontrol(targetInd)

        controlAction.drive.steering_angle = delta
        controlAction.drive.speed = TARGETSPEED
        ################
        if len(waypoints.xList) != 0:
            rospy.loginfo("target ind in node: " + str(targetInd))
            targetPose.pose.position.x = waypoints.xList[targetInd]
            targetPose.pose.position.y = waypoints.yList[targetInd]
            # targetPose.pose.position.x = waypoints.waypoints.poses[-1].pose.position.x
            # targetPose.pose.position.y = waypoints.waypoints.poses[-1].pose.position.y
            targetPose.pose.position.z = -0.5
            targetPose.action = Marker.ADD
            targetPose.header.frame_id = "velodyne"
            targetPose.header.stamp = rospy.Time.now()
            targetPose.type = Marker.SPHERE
            targetPose.id = 0
            targetPose.color.r = 1.0
            targetPose.ns = "targetPoint"
            targetPose.color.a = 1.0
            targetPose.pose.orientation.w = 1.0
            targetPose.scale.x = 0.5
            targetPose.scale.y = 0.5
            targetPose.scale.z = 0.5
        ################
        controlActionPub.publish(controlAction)
        targetPosePub.publish(targetPose)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()

    except rospy.ROSInterruptException:
        pass
