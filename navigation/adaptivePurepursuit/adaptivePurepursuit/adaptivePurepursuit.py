import rclpy
import math


class AdaptivePurePursuit:
    def __init__(self):

        #functions initializations
        self.velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.target_speed = 0.0
        self.prev_error = 0.0
        self.error_sum = 0.0
        self.waypoints = []
        self.pathFlag = False
        self.firstFlag = True
        self.target_index = 0
        self.steering_angle = 0.0
        self.first_element = []


        #From launch file
        self.kp = rclpy.Node.get_parameter_or("/gains/proportional")
        self.ki = rclpy.Node.get_parameter_or("/gains/integral")
        self.kd = rclpy.Node.get_parameter_or("/gains/differential")
        self.dt = rclpy.Node.get_parameter_or("/time_step")
        self.lookahead_distance = rclpy.Node.get_parameter_or("/gains/lookahead_distance")
        self.lookaheadconstant = rclpy.Node.get_parameter_or("/look_ahead_constant")
        self.gain = rclpy.Node.get_parameter_or("/gain")
        self.minspeed = rclpy.Node.get_parameter_or("/speed/min")
        self.maxspeed = rclpy.Node.get_parameter_or("/speed/max")
        self.speed_constant = rclpy.Node.get_parameter_or("/speed_constant")

    @staticmethod
    def calculate_distance(point1: list, point2: list):
        delta_x = point2[0] - point1[0]
        delta_y = point2[1] - point1[1]
        return math.sqrt(delta_x ** 2 + delta_y ** 2)


    def search_target_point(self):
        min_distance = float('inf')
        if self.firstFlag:
            for i , waypoint in enumerate(self.waypoints):
                distance = self.calculate_distance(self.state[:2], waypoint)
                if distance < min_distance:
                    min_distance = distance
                    self.target_index = i
                    self.firstFlag = False
        
        for i in range(self.target_index,len(self.waypoints) - 1):
            distance = self.calculate_distance(self.state[:2], self.waypoints[i])
            if distance > self.lookahead_distance :
                self.target_index = i
                if self.x > self.first_element[self.target_index]:
                    continue
                else:
                    break
        return self.target_index
    

    def adaptivePurepursuit(self):
        self.lookahead_distance = self.velocity * self.gain + self.lookaheadconstant
        self.target_index = self.search_target_point()
        target_waypoint = self.waypoints[self.target_index]
        tx, ty = target_waypoint
        dx = tx - self.x
        dy = ty - self.y
        alpha = math.atan2(dy, dx) - self.yaw
        lookahead_angle = math.atan2(2 * 0.5 * math.sin(alpha) / self.lookahead_distance, 1)
        self.steering_angle = math.degrees(lookahead_angle)
        self.steering_angle = max(-0.5, min(0.5, lookahead_angle))
        return self.steering_angle
    

    def speedControl(self, steering_angle: float) -> float:
        self.target_speed: float = self.speed_constant / (abs(steering_angle) + 0.001)
        self.target_speed = min(self.target_speed,self.maxspeed)
        self.target_speed = max(self.target_speed,self.minspeed)
        return self.target_speed
    

    def pid_controller(self,steering):
        self.target_speed = self.speedControl(steering)
        error = self.target_speed - self.velocity
        p_term = self.kp * error
        self.error_sum += error
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.prev_error) / self.dt
        control_signal = p_term + i_term + d_term
        self.prev_error = error
        control_signal= max(-1.0, min(1.0, control_signal))
        return control_signal