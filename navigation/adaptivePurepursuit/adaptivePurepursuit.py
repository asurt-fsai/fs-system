import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry, Path
import math
from tf_transformations import euler_from_quaternion
import numpy as np




lookaheadconstant = 2.0
Gain = 0.3
minspeed = 0.5
maxspeed = 3.5



class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.steer_pub = self.create_publisher(Float32, 'steer', 10)
        self.throttle_pub = self.create_publisher(Float32, 'throttle', 10)
        self.state_sub = self.create_subscription(Odometry, 'state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, 'path', self.path_callback, 10)

        self.timer = self.create_timer(0.1,self.publish_control_signals)

        self.i = 0
        self.target_speed = 0.0

        self.X = 0.0
        self.Y = 0.0
        self.yaw = 0.0
        self.error_sum = 0.0
        self.velocity = 0.0
        self.state = (self.X, self.Y, self.yaw)
        self.waypoints = []
        self.lookahead_distance = 4.2
        
        self.dt = 0.1

        self.kp = 0.5
        self.ki = 1.0
        self.kd = 0.5
        self.prev_error = 0.0
        self.pathFlag = False
        

    def state_callback(self, state: Odometry):
        Vx = state.twist.twist.linear.x
        Vy = state.twist.twist.linear.y
        self.velocity = math.sqrt(Vx ** 2 + Vy ** 2)
        self.X = state.pose.pose.position.x
        self.Y = state.pose.pose.position.y
        orientation_list = [
            state.pose.pose.orientation.x,
            state.pose.pose.orientation.y,
            state.pose.pose.orientation.z,
            state.pose.pose.orientation.w
        ]
        _, _, self.yaw = euler_from_quaternion(orientation_list)
        
        
        self.lookahead_distance = self.velocity * Gain + lookaheadconstant

    def path_callback(self, path: Path):
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path.poses]
        self.pathFlag = True
    

    def pid_controller(self,steering):
        self.target_speed = self.proportionalControl(steering)
        error = self.target_speed - self.velocity
        p_term = self.kp * error
        self.error_sum += error
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.prev_error) / self.dt
        control_signal = p_term + i_term + d_term
        self.prev_error = error
        control_signal= max(-1.0, min(1.0, control_signal))
        acceleration = control_signal
        return acceleration


    def search_target_point(self):
    
        min_distance = float('inf')
        target_index = None
        
        for self.i, waypoint in enumerate(self.waypoints):
            distance = self.calculate_distance([self.X,self.Y], waypoint)
            if distance < min_distance:
                min_distance = distance
                min_index = self.i
        self.lookahead_distance =  self.velocity * Gain + lookaheadconstant
        for self.i in range(min_index,len(self.waypoints) - 1):
            distance = self.calculate_distance([self.X,self.Y], waypoint)
            if distance > self.lookahead_distance :
                target_index = self.i
                return target_index
     
        for self.past_index, waypoint in enumerate(self.lookahead_distance):
            distance = self.calculate_distance([self.X,self.Y], waypoint)
            if distance == self.lookahead_distance:
                target_index = i
                self.past_index = target_index
                i += 1
                break
        return target_index
    
    @staticmethod
    def calculate_distance(point1: list, point2: list):
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        return math.sqrt(delta_x ** 2 + delta_y ** 2)

    def pure_pursuit(self):
        self.lookahead_distance = self.velocity * Gain + lookaheadconstant
        
        target_index = self.search_target_point()
        target_waypoint = self.waypoints[target_index]
        tx, ty = target_waypoint
        dx = tx - self.X
        dy = ty - self.Y
        # if target_index == len(self.waypoints) - 1:
        #         return 0
        alpha = math.atan2(dy, dx) - self.yaw
        look_ahead_angle = math.atan2(2 * 0.5 * math.sin(alpha) / self.lookahead_distance, 1)
        steering_angle = max(-0.5, min(0.5, look_ahead_angle))
        return steering_angle
    

    def proportionalControl(self, steering_angle: float) -> float:
        targetSpeed: float = (20.0/3.6) / (abs(steering_angle) + 0.001) # change steering angle to rad
        #targetSpeed: float = map(abs(steering_angle),0,30,3,0.5)
        targetSpeed = min(targetSpeed,maxspeed)
        targetSpeed = max(targetSpeed,minspeed)
        return targetSpeed
    
    def publish_control_signals(self):
        if self.pathFlag == True :
            steering_angle = Float32()
            steering_angle.data = self.pure_pursuit()*(180/math.pi)
            self.steer_pub.publish(steering_angle)
            if len(self.waypoints) > 0 and self.search_target_point == len(self.waypoints) - 1:
                throttle = 0
            throttle = Float32()
            throttle.data = self.pid_controller(steering_angle.data)
            self.throttle_pub.publish(throttle)
            
            log = "tracking waypoint: " + str(self.waypoints[self.i])
            self.get_logger().info(log)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()