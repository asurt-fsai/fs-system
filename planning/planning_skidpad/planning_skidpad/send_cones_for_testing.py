import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseArray , Pose 
from nav_msgs.msg import Odometry


class SendCones(Node):

    def __init__(self):
        super().__init__('ConesPublisher')
        self.publisher_ = self.create_publisher(PoseArray, 'cones', 10)
        self.subscription_state = self.create_subscription(Odometry, 'state', self.stateCallback, 10)
        self.subscriptions
        
        self.cone_positions = [
            [-1.5, 0.0, 1], 
            [-1.5, 5.0, 1], 
            [-1.5, 13.45, 2], 
            [-1.5, 16.55, 2], 
            [-1.5, 25.0, 1], 
            [-1.5, 30.0, 1], 
            [-1.5, 35.0, 1], 
            [-1.5, 40.0, 1], 
            [-0.5, 40.0, 1], 
            [0.5, 40.0, 1], 
            [1.5, 0.0, 1], 
            [1.5, 5.0, 1], 
            [1.5, 13.45, 2], 
            [1.5, 16.55, 2], 
            [1.5, 25.0, 1], 
            [1.5, 30.0, 1], 
            [1.5, 35.0, 1], 
            [1.5, 40.0, 1], 
            [-1.5, 15.0, 3], 
            [-2.08, 17.918, 3], 
            [-3.733, 20.392, 3], 
            [-6.207, 22.045, 3], 
            [-9.125, 22.625, 3], 
            [-12.043, 22.045, 3], 
            [-14.517, 20.392, 3], 
            [-16.17, 17.918, 3], 
            [-16.75, 15.0, 3], 
            [-16.17, 12.082, 3], 
            [-14.517, 9.608, 3], 
            [-12.043, 7.955, 3], 
            [-9.125, 7.375, 3],# 
            [-6.207, 7.955, 3], 
            [-3.733, 9.608, 3], 
            [-2.08, 12.082, 3],  
            [16.17, 17.918, 4], 
            [14.517, 20.392, 4], 
            [12.043, 22.045, 4], 
            [9.125, 22.625, 4], 
            [6.207, 22.045, 4], 
            [3.733, 20.392, 4], 
            [2.08, 17.918, 4], 
            [1.5, 15.0, 4], 
            [2.08, 12.082, 4], 
            [3.733, 9.608, 4], 
            [6.207, 7.955, 4], 
            [9.125, 7.375, 4],# 
            [12.043, 7.955, 4], 
            [14.517, 9.608, 4], 
            [16.17, 12.082, 4], 
            [16.75, 15.0, 4], 
            [-1.5, 22.399, 4],
            [-4.642, 24.633, 4], 
            [-8.373, 25.598, 4], 
            [-12.204, 25.169, 4], 
            [-15.629, 23.401, 4], 
            [-18.199, 20.528, 4], 
            [-19.574, 16.927, 4], 
            [-19.574, 13.073, 4], 
            [-18.199, 9.472, 4], 
            [-15.629, 6.599, 4], 
            [-12.204, 4.831, 4], 
            [-8.373, 4.402, 4],# 
            [-4.642, 5.367, 4], 
            [-1.5, 7.601, 4], 
            [1.5, 7.601, 3], 
            [4.642, 5.367, 3], 
            [8.373, 4.402, 3], 
            [12.204, 4.831, 3], 
            [15.629, 6.599, 3],# 
            [18.199, 9.472, 3],# 
            [19.574, 13.073, 3], 
            [19.574, 16.927, 3], 
            [18.199, 20.528, 3], 
            [15.629, 23.401, 3], 
            [12.204, 25.169, 3], 
            [8.373, 25.598, 3], 
            [4.642, 24.633, 3], 
            [1.5, 22.399, 3]]
        
        self.state=Odometry()
  
    
    def stateCallback(self, state:Odometry):
        self.state = state
        conePosition = PoseArray()
        nearCones = self.get_cones_near_pos([self.state.pose.pose.position.x,self.state.pose.pose.position.y])
        print(nearCones)
        for i in nearCones:
            cone = Pose()
            cone.position.x = float(i[0])
            cone.position.y = float(i[1])
            cone.position.z = float(i[2])
            conePosition.poses.append(cone)
        self.publisher_.publish(conePosition)

    
    def get_cones_near_pos(self,pos):
        cones_near_pos = []
        for i in self.cone_positions:
            if math.sqrt((pos[0]-i[0])**2+(pos[1]-i[1])**2) < 15 :
                cones_near_pos.append(i)
        return cones_near_pos

def main(args=None):
    rclpy.init(args=args)

    cones = SendCones()

    rclpy.spin(cones)

    cones.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()