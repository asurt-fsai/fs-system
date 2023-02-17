import rospy
from nav_msgs.msg import Path
import numpy as np
import csv


def add_point(data):
    global path_x, path_y
    with open('ipg_track.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([data.poses[-1].pose.position.x,
                        data.poses[-1].pose.position.y])


rospy.init_node("listener")
path_sub = rospy.Subscriber("/odometry/trajectory", Path, add_point)

while not (rospy.is_shutdown() == True):
    pass