import cv2
import torch
import rclpy
import numpy as np
from rclpy import Node
from kpr_model import KeypointNet
from asurt_msgs.msg import ConeImg, ConeImgArray, KeyPoints, BoundingBox, BoundingBoxes

class KeyPointRegressorNode(Node):
    
    def __init__(self):
        
        super().__init__("keypoint_regressor")
        self.get_logger().info("STARTING KEYPOINT REGRESSOR")
        
        self.declare_parameters(
            namespace='',
            parameters = [
                ('/kpr/model_path', rclpy.Parameter.Type.STRING),
                ('/kpr/bboxes', rclpy.Parameter.Type.STRING),
                ('/kpr/cropped_detections', rclpy.Parameter.Type.STRING),
                ('/kpr/keypoints_topic', rclpy.Parameter.Type.STRING )
            ]
        )
        
        self.model = None
        self.cone_subscriber = None
        self.bbox_subscriber = None
        self.kps_publisher = None
        
        self.flag = None
        self.bboxes_queue = None
        
        self.start()
        
    def start(self):
        
        
        MODEL_PATH = self.get_parameter('/kpr/model_path').get_parameter_value().string_value
        bboxes_msg_topic = self.get_parameter('/kpr/bboxes').get_parameter_value().string_value
        cone_msg_topic = self.get_parameter('/kpr/cropped_detections').get_parameter_value().string_value
        kps_msg_topic = self.get_parameter('/kpr/keypoints_topic').get_parameter_value().string_value

        self.flag = 0
        self.bboxes_queue = []
        
        self.model = KeypointNet()
        self.model.load_state_dict(torch.load(MODEL_PATH).get('model'))
        self.model.eval()
        
        self.cone_subscriber = self.create_subscription(ConeImgArray, cone_msg_topic, self.kpr_infer, 10)
        self.bbox_subscriber = self.create_subscription(BoundingBoxes, bboxes_msg_topic, self.process_Bboxes, 10)
        self.kps_publisher = self.create_publisher(KeyPoints, kps_msg_topic, 10)
    
    
    def seqto2D(self, cone_msg:ConeImg):
            
        cone_img = cone_msg.data.reshape(cone_msg.rows, cone_msg.cols)
        return cone_img
    
    
    def prep_image(self, image, target_image_size = (80, 80)):
        
        image = cv2.resize(image, target_image_size)
        image = (image.transpose((2, 0, 1)) / 255.0)[np.newaxis, :]
        
        return image
    
    def prepare_batch_of_cones(self, cones_array_msg:ConeImgArray):
        
        #This function prepares a batch of cones **from single msg** to be infered into KeyPoint Regressor
        
        cones = []
        for cone_msg in cones_array_msg:
            cone_image = self.seqto2D(cone_msg)
            cone_image = self.prep_image(cone_image)
            cones.append(cone_image)
        
        batch = np.stack(cones)
        batch = torch.from_numpy(batch).type('torch.FloatTensor')
        
        return batch
    
    def process_Bboxes(self, bboxes_msg):
        
        if(self.flag == 0):
            self.bboxes_queue = bboxes_msg.bounding_boxes
        
    
    def map_keypoints_to_global(self, kpr_output):
            
        kpr_output = kpr_output.numpy()
        
        for idx, kps in enumerate(kpr_output):
            
            bbox_info = self.bboxes_queue[idx]
            
            scale_w = bbox_info.w / 80
            scale_h = bbox_info.h / 80
            
            kps[:,0] = kps[:,0]*scale_w + bbox_info.x_min
            kps[:,1] = kps[:,1]*scale_h + bbox_info.y_min
        
        self.flag = 0
        
        return kpr_output
                
                     
         
    def prepare_keypoints_msg(self, frame_id, object_count, keypoints_array):
        
        msg = KeyPoints()
        
        msg.frame_id = frame_id
        msg.object_count = object_count
        msg.keypoints = keypoints_array.ravel()
        
        return msg
        
        
                   
 
    def kpr_infer(self, cones_array_msg):
        
        frame_id = cones_array_msg.frame_id
        object_count = cones_array_msg.object_count
        
        try:
            
            cones_batch = self.prepare_batch_of_cones(cones_array_msg)
            output = self.model(cones_batch)
            
        except:
            self.get_logger().error(f"Failed to infere keypoint regressor")
            
        keypoints = self.map_keypoints_to_global(self, output)
        
        
        #Prepare your msg
        kps_msg = self.prepare_keypoints_msg(frame_id, object_count, keypoints)
        
        #Publish It
        self.kps_publisher.publish(kps_msg)
            
        
            
        
        
def main(args=None):

    rclpy.init(args=args)
    node = KeyPointRegressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, shutting down.")
    except Exception as e:
        print(f"Caught exception: {e}") # Handle other exceptions
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()