import rospy
import math
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

#parameters
gainLH =  0.3  # look forward gain
LOOKAHEADCONSTANT =  2.0  # [m] look-ahead distance
Kp =  1.0  #acc gain 1
Kd = 0.1 #acc gain 2
dt =  0.1  # [s] time step
BaseWidth = 2.9  # [m] wheel base of vehicle
show_animation = True
#getting car state from SLAM
