import cv2
import torch
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from .kpr_model import KeypointNet
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
        
        self.bridge = None
        self.model = None
        self.cone_subscriber = None
        self.bbox_subscriber = None
        self.kps_publisher = None
        
        self.flag = None
        self.views_ids_queue = None
        self.cones_msgs_queue = None
        
        self.start()
        
    def start(self):
        
        
        MODEL_PATH = self.get_parameter('/kpr/model_path').get_parameter_value().string_value
        
        #TO BE SUBSCRIBED
        bboxes_msg_topic = self.get_parameter('/kpr/bboxes').get_parameter_value().string_value
        cone_msg_topic = self.get_parameter('/kpr/cropped_detections').get_parameter_value().string_value
        
        #TO BE PUBLISHED
        kps_msg_topic = self.get_parameter('/kpr/keypoints_topic').get_parameter_value().string_value

        self.flag = 0
        self.bboxes_queue = []
        
        self.views_ids_queue = []
        self.cones_msgs_queue = []
        
        self.bridge = CvBridge()
        self.model = KeypointNet()
        self.model.load_state_dict(torch.load(MODEL_PATH).get('model'))
        
        
        self.cone_subscriber = self.create_subscription(ConeImgArray, cone_msg_topic, self.queue_detections, 10)
        self.bbox_subscriber = self.create_subscription(BoundingBoxes, bboxes_msg_topic, self.process_Bboxes, 10)
        self.kps_publisher = self.create_publisher(KeyPoints, kps_msg_topic, 10)
        self.get_logger().info("KEYPOINT REGRESSOR STARTED")
    

    
    def queue_detections(self, cones_images_msg):
        '''
        This function queues cone images until its bbox arrives from tracker
        '''
    
        #FOR MEMORY
        if len(self.views_ids_queue)>10:
            self.views_ids_queue.pop(0)
            self.cones_msgs_queue.pop(0)
        ###########
        
        frame_id = cones_images_msg.view_id
        object_count = cones_images_msg.object_count
        self.views_ids_queue.append(frame_id)
        self.cones_msgs_queue.append(cones_images_msg)
        self.get_logger().info(f"Recieved Detections of Frame {frame_id}")
        
        ##################################### DEBUGGING #################################################
        self.get_logger().info(f"Detecting Keypoints")
        
        if(object_count!=0):
            self.get_logger().info(f'{object_count}')
            
            results = self.kpr_infer(cones_images_msg)
        
        kps_msg = self.prepare_keypoints_msg(frame_id, object_count, list(np.zeros(object_count)), list(np.zeros(object_count)), results)
        self.get_logger().info(f"Publishing Keypoints")
        self.kps_publisher.publish(kps_msg)
        ##############################################################################
    
    # def seqto2D(self, cone_msg:ConeImg):
            
    #     cone_img = cone_msg.img.reshape(cone_msg.rows, cone_msg.cols)
    #     return cone_img
    
    
    def prep_image(self, image, target_image_size = (80, 80)):
        
        image = cv2.resize(image, target_image_size)
        image = (image.transpose((2, 0, 1)) / 255.0)
        # image = (image / 255.0)[np.newaxis, :]
        return image
    
    def prepare_batch_of_cones(self, cones_array_msg:ConeImgArray):
        
        #This function prepares a batch of cones **from single msg** to be infered into KeyPoint Regressor
        
        cones = []
        for cone_msg in cones_array_msg.imgs:
            cone_image = self.bridge.imgmsg_to_cv2(cone_msg.img, desired_encoding="rgb8")
            cone_image = self.prep_image(cone_image)
            cones.append(cone_image)
        
        batch = np.stack(cones)
        batch = torch.from_numpy(batch).type('torch.FloatTensor')
        
        return batch

    
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
                
                     
         
    def prepare_keypoints_msg(self, view_id, object_count, track_ids, classes, keypoints_array):
        
        msg = KeyPoints()
        
        msg.frame_id = view_id
        msg.object_count = object_count
        
        #FOR TESTING
        assert(len(track_ids)==object_count)
        assert(len(classes)==object_count)
        ###########################
        
        msg.keypoints = keypoints_array.ravel()
        
        
        return msg
        
                   
 
    def kpr_infer(self, cones_array_msg):
         
        try:
            cones_batch = self.prepare_batch_of_cones(cones_array_msg)
            
            self.model.eval()
            output = self.model(cones_batch)
            self.get_logger().info(f"{output[-1]}")
        except Exception as e:
            self.get_logger().error(f"Failed to infere keypoint regressor {e}")
            
        keypoints = self.map_keypoints_to_global(output[-1].detach())
        
        return keypoints
        
        
    def process_Bboxes(self, bboxes_msg):
        
        view_id = bboxes_msg.view_id
        
        idx = self.views_ids_queue.index(view_id) #GET FRAME DATA FROM QUEUE
        cones_msg = self.cones_msgs_queue[idx]
        
        object_count = cones_msg.object_count
        
        bboxes = bboxes_msg.bounding_boxes
        
        track_ids = []
        classes = []
        
        #ASSOCIATION PART
        for cone in cones_msg:
            det_id = cone.id
            
            track_id=0
            cls = -1
            
            for box in bboxes: 
                if(box.detection_id == det_id):
                    track_id = box.track_id
                    cls = box.type
                    
            track_ids.append(track_id)
            classes.append(cls)
        ##########################
        
        keypoints = self.kpr_infer(cones_msg)
        
        #Prepare your msg
        kps_msg = self.prepare_keypoints_msg(view_id, object_count, track_ids, classes, keypoints)
            
        #Publish It
        self.kps_publisher.publish(kps_msg)
        
        
            
        
            
        
        
def main(args=None):

    rclpy.init(args=args)
    node = KeyPointRegressorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, shutting down.")
        node.destroy_node()
        rclpy.shutdown()
    # except Exception as e:
    #     print(f"Caught exception: {e}") # Handle other exceptions
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
        
if __name__ == '__main__':
    main()