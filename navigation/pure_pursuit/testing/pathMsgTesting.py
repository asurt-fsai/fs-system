# pylint: disable=all
# mypy: ignore-errors
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

rospy.init_node("pathMsgTesting", anonymous=True)
pathPub = rospy.Publisher("/path", Path, queue_size=10)
path = Path()
path.header.frame_id = "map"
path.header.stamp = rospy.Time.now()
point = path.poses
pose = PoseStamped()
pose.pose.position.x = 1.0
pose.pose.position.y = 2.0
pose.pose.orientation.z = 3.0
pose2 = PoseStamped()
pose2.pose.position.x = 4.0
pose2.pose.position.y = 5.0
pose2.pose.orientation.z = 6.0
path.poses.append(pose)
path.poses.append(pose2)

if __name__ == "__main__":
    try:
        while not rospy.is_shutdown():
            pathPub.publish(path)
            rospy.sleep(1)
            print((point[0].pose.position.x))

    except rospy.ROSInterruptException:
        pass
