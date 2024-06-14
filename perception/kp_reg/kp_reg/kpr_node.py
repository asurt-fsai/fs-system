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
                ('/kpr/bboxes_yolo', rclpy.Parameter.Type.STRING),
                ('/kpr/cropped_detections', rclpy.Parameter.Type.STRING),
                ('/kpr/keypoints_topic', rclpy.Parameter.Type.STRING )
            ]
        )
        
        self.bridge = None
        self.model = None
        self.cone_subscriber = None
        self.bbox_yolo_subscriber = None
        self.tracks_subscriber = None
        self.kps_publisher = None
        self.bboxes = None
        self.keypoints = None
        self.bboxes_flag = None
        self.keypoints_flag = None
        
        self.start()
        
    def start(self):
        
        
        MODEL_PATH = self.get_parameter('/kpr/model_path').get_parameter_value().string_value
        
        #TO BE SUBSCRIBED
        # tracks_msg_topic = self.get_parameter('/kpr/bboxes').get_parameter_value().string_value
        bboxes_msg_topic = self.get_parameter('/kpr/bboxes_yolo').get_parameter_value().string_value
        cone_msg_topic = self.get_parameter('/kpr/cropped_detections').get_parameter_value().string_value
        
        #TO BE PUBLISHED
        kps_msg_topic = self.get_parameter('/kpr/keypoints_topic').get_parameter_value().string_value

        self.bboxes_flag = 0
        self.keypoints_flag = 0
        
    
        
        
        self.bridge = CvBridge()
        self.model = KeypointNet()
        self.model.load_state_dict(torch.load(MODEL_PATH).get('model'))
        self.model.eval()
        
        self.cone_subscriber = self.create_subscription(ConeImgArray, cone_msg_topic, self.callback_keypoints, 10)
        self.bbox_yolo_subscriber = self.create_subscription(BoundingBoxes, bboxes_msg_topic, self.callback_bboxes, 10)
        
        # self.tracks_subscriber = self.create_subscription(BoundingBoxes, tracks_msg_topic, self.assign_track_id, 10)
        
        self.kps_publisher = self.create_publisher(KeyPoints, kps_msg_topic, 10)
        self.get_logger().info("KEYPOINT REGRESSOR STARTED")
    
    
    
    def callback_bboxes(self, bboxes):
        self.bboxes = bboxes
        self.bboxes_flag = 1
        self.get_logger().info(f"Recieved BBoxes of Frame {bboxes.view_id}")
        self.callback_send_kps()
        
        
    
    def callback_keypoints(self, cones_images_msg):
        
        frame_id = cones_images_msg.view_id
        object_count = cones_images_msg.object_count
        
        self.get_logger().info(f"Recieved Cones Images of Frame {frame_id}")
        
        
        ##################################### KEYPOINTS #################################################
        self.get_logger().info(f"Detecting Keypoints")
        if(object_count>0):
            
            results = self.kpr_infer(cones_images_msg)
            self.keypoints_flag = 1
            self.keypoints = results
            self.callback_send_kps()
    
        else:
            self.get_logger().info(f"NO OBJECTS DETECTED IN FRAME {frame_id}")
        ##############################################################################
    
    def callback_send_kps(self):
        if self.bboxes_flag and self.keypoints_flag:
            
            frame_id = self.bboxes.view_id
            object_count = self.keypoints.shape[0]
            
            object_count_from_yolo = self.bboxes.object_count
            assert(object_count==object_count_from_yolo)
            
            bboxes_info = self.bboxes.bounding_boxes
            
            keypoints, classes = self.map_keypoints_to_bbox(self.keypoints, bboxes_info)
                
            kps_msg = self.prepare_keypoints_msg(frame_id, object_count, list(np.zeros(object_count)), classes, keypoints)
            self.get_logger().info(f"Publishing Keypoints")
            self.kps_publisher.publish(kps_msg)
            
            self.keypoints_flag = 0
            self.bboxes_flag = 0
            
            
            
    
################################################################################################################
    
    def dequeue_kps_from_yolo(self, bboxes):
        '''
        This function queues bboxe coming from yolo until its bbox arrives from tracker
        '''
        
        frame_id = bboxes.view_id
        
        # DEQUEUE KPS
        if len(self.frame_ids_of_keypoints):
            
            idx = self.frame_ids_of_keypoints.index(frame_id) #GET FRAME DATA FROM QUEUE
            kps = self.keypoints[idx]
            
            
            object_count = kps.shape[0]
            
            object_count_from_yolo = bboxes.object_count
            assert(object_count==object_count_from_yolo)
            
            bboxes_info = bboxes.bounding_boxes
            
            # #FOR MEMORY
            # if len(self.views_ids_queue)>10:
            #     self.views_ids_queue.pop(0)
            #     self.cones_msgs_queue.pop(0)
            # ###########
            
            keypoints, classes = self.map_keypoints_to_bbox(kps, bboxes_info)
                
            kps_msg = self.prepare_keypoints_msg(frame_id, object_count, list(np.zeros(object_count)), classes, keypoints)
            self.get_logger().info(f"Publishing Keypoints")
            self.kps_publisher.publish(kps_msg)
    
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
        batch = torch.from_numpy(batch).type('torch.FloatTensor').contiguous()
        
        return batch

    
    def map_keypoints_to_bbox(self, keypoints, bboxes):
            
        # self.get_logger().info(keypoints.shape)
        keypoints = keypoints.numpy()
        
        classes = []
        
        for idx, kps in enumerate(keypoints):
            
            bbox_info = bboxes[idx]
            classes.append(bbox_info.type)
            scale_w = bbox_info.width / 80
            scale_h = bbox_info.height / 80
            
            kps[:,0] = kps[:,0]*scale_w + bbox_info.xmin
            kps[:,1] = kps[:,1]*scale_h + bbox_info.ymin
            
            
        
        return keypoints, classes
                
                     
         
    def prepare_keypoints_msg(self, view_id, object_count, track_ids, classes, keypoints_array):
        
        msg = KeyPoints()
        
        msg.view_id = view_id
        msg.object_count = object_count
        
        #FOR TESTING
        assert(len(track_ids)==object_count)
        assert(len(classes)==object_count)
        ###########################
        
        
        # self.get_logger().info(keypoints_array)
        msg.keypoints = keypoints_array.ravel().tolist()
        
        
        return msg
        
                   
 
    def kpr_infer(self, cones_array_msg):
         
        try:
            cones_batch = self.prepare_batch_of_cones(cones_array_msg)
            with torch.no_grad():
                keypoints = self.model(cones_batch)[-1]
    
        except Exception as e:
            self.get_logger().error(f"Failed to infere keypoint regressor {e}")
            self.get_logger().error(f"Batch Shape {cones_batch.shape}")
            # self.get_logger().error(f"Batch Shape {keypoints.shape}")
        
        return keypoints
        
        
    def assign_track_id(self, bboxes_msg):
        
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