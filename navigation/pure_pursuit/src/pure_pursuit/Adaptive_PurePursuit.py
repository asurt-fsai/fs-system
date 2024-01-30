import numpy as np 
import matplotlib.pyplot as plt

class PurePursuitController:
    def __init__(self,lookahead_distance):
        self.lookahead_distance = lookahead_distance

    def compute_steering(self,path,position,orientation):
        closest_point, closest_index = self._find_closest_point(path,position)
        target_point, target_index = self._find_target_point(path,closest_index, self.lookahead_distance)
        steering_angle = self._compute_steering_angle(position,orientation,target_point)
        return steering_angle
    
    def _find_closest_point(self,path,position):
        distances = np.linalg.norm(path - position, axis = 1)
        closest_index = np.argmin(distances)
        return path[closest_index], closest_index
    
    def _find_target_point(self,path,closest_index,lookahead_distance):
        distance = 0.0
        target_index = closest_index
        while distance < lookahead_distance and target_index < len(path) - 1:
            segment_length = np.linalg.norm(path[target_index + 1] - path[target_index])
            if distance + segment_length < lookahead_distance:
                distance += segment_length
                target_index += 1
            else:
                break

        remaining_distance = lookahead_distance - distance
        direction = (path[target_index + 1] - path[target_index]) / np.linalg.norm(path[target_index + 1] - path[target_index])
        target_point = path[target_index] + direction * remaining_distance
        return target_point,target_index
    
    def _compute_steering_angle(self, position, orientation, target_point):
        target_vector = target_point - position
        target_angle = np.arctan2(target_vector[1],target_vector[0])
        steering_angle = target_angle - orientation
        return steering_angle
    

path = np.array([[0,0],[1,1],[2,2],[3,1],[4,0]])
position = np.array([0.5,0.5])
orientation = 0.0
lookahead_distance = 0.5

controller = PurePursuitController(lookahead_distance)
steering_angle = controller.compute_steering(path,position,orientation)
print("Steering angle :", steering_angle)

plt.plot(path[:, 0], path[:,1], '-o',label = 'path')
plt.plot(position[0], position[1],'ro', label = 'Car')
plt.quiver(position[0], position[1],np.cos(orientation),np.sin(orientation), angles = 'xy', scale = 1 , color = 'r', label = 'orientation')
plt.legend()
plt.axis('equal')
plt.show()