import cv2
import torch
import rclpy
import numpy as np
from rclpy import Node
from kpr_model import KeypointNet

PT_DIR = "kpr.pt"


def prep_image(image,target_image_size = (80, 80)):
    h,w,_ = image.shape
    image = cv2.resize(image, target_image_size)
    image = (image.transpose((2, 0, 1)) / 255.0)[np.newaxis, :]
    image = torch.from_numpy(image).type('torch.FloatTensor')
    
    return image

class KeyPointRegressorNode(Node):
    
    def __init__(self):
        
        super().__init__("keypoint_regressor")
        
        self.model = KeypointNet()
        self.model.load_state_dict(torch.load(PT_DIR).get('model'))
        self.model.eval()
        
        self.cone_subscriber = self.create_subscription("cropped_cone_msg_type", '/topic/name', self.kpr_infer, 10)
        self.kps_publisher = self.create_publisher("keypoints_array_msg_type", '/topic/name', 10)
        
    def kpr_infer(self, cone_img_msg):
        
        cone_img = prep_image(cone_img_msg)
        output = self.model(cone_img)
        
        #Prepare your msg
        kps_msg = output
        
        #Publish It
        self.kps_publisher.publish(kps_msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = KeyPointRegressorNode()
    rclpy.spin(node)
    rclpy.shutdown()