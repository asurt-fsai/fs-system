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


class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, currentSpeed=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rear_x = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
    def update(self, acc, delta):
        self.x += self.currentSpeed * math.cos(self.yaw) * dt
        self.y += self.currentSpeed * math.sin(self.yaw) * dt
        self.yaw += self.currentSpeed / BaseWidth * math.tan(delta) * dt   
        self.currentSpeed += acc * dt  
        self.rear_x = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

#storing the states of the vehicle gotten from SLAM
class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.currentSpeed = []
        self.time = []
        
    def update(self, time, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.currentSpeed.append(state.currentSpeed)
        self.time.append(time)
        

#storing the waypoints from path planning
class WayPoints:
    def __init__(self, X_coordinates, Y_Coordinates):
        self.X_coordinates : list = X_coordinates  #rospy.Subscriber("X coordinate", PoseStamped, callback=self.callback)
        self.Y_coordinates : list = Y_Coordinates #rospy.Subscriber("y coordinate", PoseStamped, callback=self.callback)
        self.old_nearest_point_index = None
    
    def update(self, X_coordinates, Y_coordinates):
        self.X_coordinates.append(X_coordinates)
        self.Y_coordinates.append(Y_coordinates)

    def search_target_index(self, state):

        #(to be optimized)
        if self.old_nearest_point_index is None:
            # search nearest point index 
            self.dx = [state.rear_x - iX_coordinates for iX_coordinates in self.X_coordinates]
            self.dy = [state.rear_y - iY_coordinates for iY_coordinates in self.Y_coordinates]
            self.d = np.hypot(self.dx, self.dy)
            ind = np.argmin(self.d) 
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.X_coordinates[ind],self.Y_coordinates[ind])
            if distance_this_index > 1.0:
                rospy.sleep(0.02)
            while True:
                
                self.distance_next_index = state.calc_distance(self.X_coordinates[ind + 1],self.Y_coordinates[ind + 1])
                if distance_this_index < self.distance_next_index:
                    break
                if (ind + 1) < len(self.X_coordinates):
                    ind = ind + 1  
                else :
                    ind = ind
                distance_this_index = self.distance_next_index
            self.old_nearest_point_index = ind

        
        Lookahead = gainLH * state.currentSpeed + LOOKAHEADCONSTANT  # update look ahead distance
        # search look ahead target point index
        while Lookahead > state.calc_distance(self.X_coordinates[ind], self.Y_coordinates[ind]):
            if (ind + 1) >= len(self.X_coordinates):
                break  # not exceed goal
            ind += 1

        return ind, Lookahead # return the index of the target point and the look ahead distance
        