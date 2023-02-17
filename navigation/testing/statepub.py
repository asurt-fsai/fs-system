#!/usr/bin/env python3
import math
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry
#from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt


#parameters
gainLH = rospy.get_param("/gains/look_forward")   # look forward gain 
LOOKAHEADCONSTANT = rospy.get_param("/look_ahead_constant") # look ahead constant
Kp =  rospy.get_param("/gains/propotional") # propotional gain
Kd = rospy.get_param("/gains/differential") # differential gain
Ki = rospy.get_param("/gains/integral") # integral gain
dt = 0.1 #rospy.get_param("/time_step") # [s] time step 
BaseWidth = rospy.get_param("/base_width") # [m] car length
SHOWANIMATION = True
max_Speed = 3/3.6
min_Speed = 0.0
#self.velocitrajY_publisher = rospy.Publisher ('/turtle1/cmd_vel', Twist, queue_size=10)
    
#self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

#getting car state from SLAM
class State:
    """
    
    state of the vehicle gotten from SLAM
    
    """
    def __init__(self, x:float, y:float, yaw:float, currentSpeed:float) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        #rospy.Subscriber("/control_actions", Pose, self.update)
        self.rearX = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
    def update(self, control_actions:Pose)  :
        """
        
        update the state of the vehicle
        
        """
        #data = control_actions
        acc = control_actions.position.x
        delta = control_actions.position.y
        self.x += self.currentSpeed * math.cos(self.yaw) * dt
        self.y += self.currentSpeed * math.sin(self.yaw) * dt
        self.yaw += self.currentSpeed / BaseWidth * math.tan(delta) * dt   
        # if self.currentSpeed > max_Speed:
        #     self.currentSpeed = max_Speed
        # elif self.currentSpeed < min_Speed:
        #     self.currentSpeed = min_Speed
        self.currentSpeed += acc * dt  
        self.rearX = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
       
    def calcDistance(self, point_x:float, point_y:float) -> float:
        """
        
        calculate the distance between the vehicle and the target point
        
        """
        distanceX = self.rearX - point_x
        distanceY = self.rearY - point_y
        return math.hypot(distanceX, distanceY)

#storing the states of the vehicle gotten from SLAM
class States:
    def __init__(self) -> None:
        self.x = [0]
        self.y = [0]
        self.yaw = [0]
        self.currentSpeed = [0]
        self.time = [0]
    #yaw[0]
    def update(self, time, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.currentSpeed.append(state.currentSpeed)
        self.time.append(time)
        



    
def main():
    
    rospy.init_node ('state_node', anonymous=True)
    # pose = Pose()
   
    # X_coordinate = pose.position.x #np.arange(0, 100, 0.5)
    # Y_coordinate = pose.position.y  #[math.sin(ix / 5.0) * ix / 2.0 for ix in X_coordinates]
    # Z_coordinate = pose.position.z 
   
   
    
    
    T = 100.0  # max simulation time
    
    # initial state
    state = State(0.0, 0.0, 0.0, 0.0)
    #rospy.Subscriber("/control_actions", Pose, state.update)
    state_publisher = rospy.Publisher("/trajectory", Pose, queue_size=10)
    newpose = Pose()
    time = 0.0
    states = States()
    
    
    rate= rospy.Rate(10)
    while T >= time :#or lastIndex >= target_ind:
        rospy.Subscriber("/control_actions", Pose, callback=state.update)
        newpose.position.x = state.x
        newpose.position.y = state.y
        newpose.position.z = state.currentSpeed
        newpose.orientation.x = state.yaw
        
        
        state_publisher.publish(newpose)

        

        states.update(time, state)
        time += dt
        #clearance = state.calcDistance(target_course.X_coordinates[-1], target_course.Y_coordinates[-1])
        #print(waypoints.searchTargetindex(state))
        #print(waypoints.X_coordinates)
        #print("target_x:",round(target_course.X_coordinates[target_ind],2),"my_x:",round(state.x,2),
         #"target_y:" ,round(target_course.Y_coordinates[target_ind],2), "my_y:", round(state.y,2),"steer:",round(di,4),"speed:", round(state.currentSpeed,2),"ind:", target_ind, "target_Speed:"
         #, target_speed,"time:", round(time,2), "clearance:", round(clearance,2))
        
        #states.update(time, state)        
       
        
            
        rate.sleep()
    # print("All x states:" , states.x) 
    # print("All y states:", states.y)
    # print("All Speed states:", states.currentSpeed)

    # assert lastIndex >= target_ind, "Cannot reach goal"
    
   


if __name__ == '__main__':
    #rospy.wait_for_message('/waypoints', Pose)

    main()
    
  
        