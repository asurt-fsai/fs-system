# TF Helper
Includes utilities for use throughout the system including:
* tf_helper class
* decorator to use mutex locks
* class for the status heartbeat
* encoders and parsers for the Landmark and Path messages

## Lidar Orient Node
This node is responsible for fetching the pointcloud from the lidar and publishing it in a correct horizontal orientation (not tilted) on a different topic. This is to save computation instead of having multiple modules having to do this same computation.
