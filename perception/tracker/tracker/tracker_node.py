#!/usr/bin/env python3
import numpy as np
import rclpy
from .sort_tracker import Sort

from rclpy.node import Node
from asurt_msgs.msg import BoundingBoxes, BoundingBox

class TrackerNode(Node):
    
    def __init__(self):

        super().__init__("tracker_node")
        self.get_logger().info("STARTING TRACKER NODE")

        #declaring params
        self.declare_parameters(
            namespace='',
            parameters=[
                ('/tracker/tracks', rclpy.Parameter.Type.STRING),
                ('/yolov8/detections', rclpy.Parameter.Type.STRING),
                ('/tracker/max_age', rclpy.Parameter.Type.DOUBLE),
                ('/tracker/min_hits', rclpy.Parameter.Type.DOUBLE),
                ('/tracker/min_iou_threshold', rclpy.Parameter.Type.DOUBLE)
            ]
        )

        self.classes = ["blue_cone", "yellow_cone", "large_orange_cone", "orange_cone"] # Class names from yolo
        self.trackers = []
        
        self.dets_subscriber = None
        self.tracks_publisher = None

        self.start()

    def start(self):
        
        #fetch parameters from launch file
        tracks_topic = self.get_parameter('/tracker/tracks').get_parameter_value().string_value
        detections_topic = self.get_parameter('/yolov8/detections').get_parameter_value().string_value
        
        #define subscriber & publisher
        self.dets_subscriber = self.create_subscription(BoundingBoxes, detections_topic, self.callback_tracker, 10)
        self.tracks_publisher = self.create_publisher(BoundingBoxes, tracks_topic, 10)

        #initializing trackers & fetching their hyperparams
        max_age = self.get_parameter('/tracker/max_age').get_parameter_value().double_value
        min_hits = self.get_parameter('/tracker/min_hits').get_parameter_value().double_value
        min_iou_thresh = self.get_parameter('/tracker/min_iou_threshold').get_parameter_value().double_value

        # Create a tracker for each class of cones
        for _ in range(len(self.classes)):
            self.trackers.append(Sort(max_age, min_hits, min_iou_thresh))
    
    def parseBboxes(self, bounding_boxes):

        detections = [[] for _ in range(len(self.classes))]
        detection_mapping = [{} for _ in range(len(self.classes))]  

        for box in bounding_boxes:
            try:
                box_type_index = self.classes.index(box.type)  # Get the index of the box type in the classes list
                box_to_add = [box.xmin, box.ymin, box.xmax, box.ymax, box.probability]
                detections[box_type_index].append(box_to_add)

                box_id = box.detection_id # Get the detection ID of the box
                # The key is the index of the box in its class list, and the value is the box ID
                detection_index = len(detections[box_type_index]) - 1
                detection_mapping[box_type_index][detection_index] = box_id
            except ValueError:
                self.get_logger().warn("Class {} is unidentified. It will be ignored by the tracker.".format(box.type))
            
        return detections, detection_mapping

    def callback_tracker(self, msg: BoundingBoxes):

        #parse bounding boxes
        detections, detections_mapping = self.parseBboxes(msg.bounding_boxes)

        #filter detections

        tracks = BoundingBoxes()
        tracks.frame_id = msg.frame_id
        tracks.object_count = msg.object_count

        for idx, dets in enumerate(detections):
            if len(dets) > 0:
                filtered = self.trackers[idx].update(np.array(dets))
            else:
                filtered = self.trackers[idx].update()

            #process tracks and add to message
            if (len(filtered) > 0):
                for f in filtered:
                    track = BoundingBox()
                    track.probability = 0
                    track.type = self.classes[idx]

                    track.xmin, track.ymin, track.xmax, track.ymax = np.array(f[:4], dtype=int)
                    track.x_center = (f[2] + f[0]) // 2
                    track.y_center = (f[3] + f[1]) // 2
                    track.width = f[2] - f[0]
                    track.height = f[3] - f[1]

                    track.detection_id = detections_mapping[idx][f[5]]
                    track.track_id = f[4]

                    if track.width>5 and track.height>5:
                        tracks.bounding_boxes.append(track)

        #publish tracks
        self.tracks_publisher.publish(tracks)
        self.get_logger().info('Published tracks' + msg.header.frame_id)

def main(args=None):

    rclpy.init(args=args)
    node = TrackerNode()
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