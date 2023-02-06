#!/usr/bin/env python3
import math
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

#parameters
gainLH = rospy.get_param("/gains/look_forward")   # look forward gain 
LOOKAHEADCONSTANT = rospy.get_param("/look_ahead_constant") # look ahead constant
Kp =  rospy.get_param("/gains/propotional") # propotional gain
Kd = rospy.get_param("/gains/differential") # differential gain
dt = rospy.get_param("/time_step") # [s] time step 
BaseWidth = rospy.get_param("/base_width") # [m] car length
show_animation = True

#self.velocity_publisher = rospy.Publisher ('/turtle1/cmd_vel', Twist, queue_size=10)
    
#self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

#getting car state from SLAM
class State:
    """
    
    state of the vehicle gotten from SLAM
    
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, currentSpeed=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rear_x = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
    def update(self, acc:float, delta:float) -> None:
        """
        
        update the state of the vehicle
        
        """
        self.x += self.currentSpeed * math.cos(self.yaw) * dt
        self.y += self.currentSpeed * math.sin(self.yaw) * dt
        self.yaw += self.currentSpeed / BaseWidth * math.tan(delta) * dt   
        self.currentSpeed += acc * dt  
        self.rear_x = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
        return None
    def calcDistance(self, point_x:float, point_y:float) -> float:
        """
        
        calculate the distance between the vehicle and the target point
        
        """
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
        


class WayPoints: 
    """
    
    storing the waypoints from path planning
    
    """

    def __init__(self, X_coordinates:list, Y_Coordinates:list, Z_coordinates:list):
        self.waypoint_sub = rospy.Subscriber("waypoints", Pose, self.update)
        self.X_coordinates : list = X_coordinates  #rospy.Subscriber("X coordinate", PoseStamped, callback=self.callback)
        self.Y_coordinates : list = Y_Coordinates #rospy.Subscriber("y coordinate", PoseStamped, callback=self.callback)
        self.old_nearest_point_index = None
        self.pose = Pose()
        
    
    def update(self, data:Pose):
        """
        used to update the waypoints
        """
        self.pose = data
        self.pose.position.y = self.pose.position.y
        self.pose.position.x = self.pose.position.x
        self.X_coordinates.append(self.pose.position.x)
        self.Y_coordinates.append(self.pose.position.y)
        
        

    def search_target_index(self, state):
        """
        
        search nearest point index and target point index
        
        """
        
        if self.old_nearest_point_index is None:
            # search nearest point index 
            self.dx = [state.rear_x - iX_coordinates for iX_coordinates in self.X_coordinates]
            self.dy = [state.rear_y - iY_coordinates for iY_coordinates in self.Y_coordinates]
            self.d = np.hypot(self.dx, self.dy)
            ind = np.argmin(self.d) 
            self.old_nearest_point_index = ind
        else:
            ind:int = self.old_nearest_point_index
            distance_this_index = state.calcDistance(self.X_coordinates[ind],self.Y_coordinates[ind])
            
            while True:
                if ind +1 == len(self.X_coordinates):
                    break
                self.distance_next_index = state.calcDistance(self.X_coordinates[ind + 1],self.Y_coordinates[ind + 1])
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
        while Lookahead > state.calcDistance(self.X_coordinates[ind], self.Y_coordinates[ind]):
            if (ind + 1) >= len(self.X_coordinates):
                break  # not exceed goal
            ind += 1

        return ind, Lookahead # return the index of the target point and the look ahead distance
        
def pure_pursuit_steer_control(state:State, trajectory:WayPoints, pind):
    ind, Lf = trajectory.search_target_index(state)
    tx = ty = 0
    if pind >= ind:
        ind = pind

    if ind < len(trajectory.X_coordinates):
        tx = trajectory.X_coordinates[ind]
        ty = trajectory.Y_coordinates[ind]
    else:  # toward goal
        tx = trajectory.X_coordinates[-1]
        ty = trajectory.Y_coordinates[-1]
        ind = len(trajectory.X_coordinates) - 1
    
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * BaseWidth * math.sin(alpha) / Lf, 1.0)

    return delta, ind

def proportional_control(targetSpeed, currentSpeed):  #longitudinal controller
    
    acc = Kp*(targetSpeed - currentSpeed) + Kd*((targetSpeed-currentSpeed)/dt )
    return acc
    
def main():
    
    rospy.init_node ('purepursuit_controller', anonymous=True)
    # pose = Pose()
    X_coordinates = [0.0]
    Y_coordinates = [0.0]
    Z_coordinates = [0.0]
    # X_coordinate = pose.position.x #np.arange(0, 100, 0.5)
    # Y_coordinate = pose.position.y  #[math.sin(ix / 5.0) * ix / 2.0 for ix in X_coordinates]
    # Z_coordinate = pose.position.z 
    waypoints = WayPoints(X_coordinates, Y_coordinates, Z_coordinates)
    

    target_speed = (20.0 / 3.6) #[m/s]
    
    T = 100.0  # max simulation time
    
    # initial state
    state = State(x=(waypoints.pose.position.x-1.0), y=(waypoints.pose.position.y-1), yaw=0.0, currentSpeed=0.0)
    
    lastIndex = len(X_coordinates) - 1
    time = 0.0
    states = States()
    states.update(time, state)
    #
    target_course = WayPoints(X_coordinates, Y_coordinates, Z_coordinates) 
    target_ind, _ = target_course.search_target_index(state)
    
    rate= rospy.Rate(1/dt)
    
    while T >= time :#or lastIndex >= target_ind:
       
        acc = proportional_control(target_speed, state.currentSpeed)  #longitudinal controller
        di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind) #lateral controller
        state.update(acc, di)   #update the state of the car
        
        target_speed = (20.0/ 3.6)/(abs(di) *4)  # [m/s]
        # waypoints.update(pose)
        if target_speed <= 15/3.6:  # min speed
            target_speed = 15/3.6
        if target_speed >= 60/3.6:   #max speed
            target_speed = 60/3.6
        time += dt
        clearance = state.calcDistance(target_course.X_coordinates[-1], target_course.Y_coordinates[-1])
        print(waypoints.X_coordinates)
        #print("target_x:",round(target_course.X_coordinates[target_ind],2),"my_x:",round(state.x,2),
         #"target_y:" ,round(target_course.Y_coordinates[target_ind],2), "my_y:", round(state.y,2),"steer:",round(di,4),"speed:", round(state.currentSpeed,2),"ind:", target_ind, "target_Speed:"
         #, target_speed,"time:", round(time,2), "clearance:", round(clearance,2))
        
        states.update(time, state)        
        if show_animation:  
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            plt.plot(X_coordinates, Y_coordinates, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(X_coordinates[target_ind], Y_coordinates[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.currentSpeed * 3.6)[:4])
            plt.pause(0.001)
        
        if clearance <= 1.4:
            # goal_reached = True
            print("Goal Reached")
            break
        
            
        rate.sleep()
    

    # assert lastIndex >= target_ind, "Cannot reach goal"
    
    if show_animation:  
        plt.cla()
        plt.plot(X_coordinates, Y_coordinates, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.time, [iv * 3.6 for iv in states.currentSpeed], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


if __name__ == '__main__':
    main()
    
  
        