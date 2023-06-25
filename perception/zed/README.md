### ZED
This module contains the code for using the ZED stereo camera with ROS. It is based on the [ZED ROS wrapper] which extracts disparity images and then this module calculates 3d positions of the cones detected by YOLO.

To use this module, you need to install the ZED SDK. You can do this by following the instructions [here](https://www.stereolabs.com/docs/installation/ubuntu/). To install the ROS wrapper, follow the instructions [here](https://www.stereolabs.com/docs/ros/).

After installing the SDK and the ROS wrapper, you can run the following command to start the ZED camera:
```
roslaunch zed_wrapper zed.launch
```
This will start the camera and publish the disparity images to the topic `/zed/zed_node/disparity/disparity_image`.

Then, you can run the following command to start this module:
```
roslaunch zed zed_cones.launch
```
This will subscribe to the `/zed/zed_node/disparity/disparity_image` topic and calculate the 3d positions of the cones published by YOLO on `/darknet_ros/bounding_boxes` . Then, it will publish the 3d positions as LandmarkArray msg to the topic `zed/detected_cones`.


To visualize the output open rviz and add a MarkerArray subscribed to the topic `/zed/detected_cones_markers`.
