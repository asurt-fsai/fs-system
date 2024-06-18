// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/circular_buffer.hpp>
#include "imageProjection.h"

const std::string PARAM_VERTICAL_SCANS = "laser.num_vertical_scans";
const std::string PARAM_HORIZONTAL_SCANS = "laser.num_horizontal_scans";
const std::string PARAM_ANGLE_BOTTOM = "laser.vertical_angle_bottom";
const std::string PARAM_ANGLE_TOP = "laser.vertical_angle_top";
const std::string PARAM_GROUND_INDEX = "laser.ground_scan_index";
const std::string PARAM_SENSOR_ANGLE = "laser.sensor_mount_angle";
const std::string PARAM_SEGMENT_THETA = "image_projection.segment_theta";
const std::string PARAM_SEGMENT_POINT = "image_projection.segment_valid_point_num";
const std::string PARAM_SEGMENT_LINE = "image_projection.segment_valid_line_num";

ImageProjection::ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel)
    : Node(name),  _output_channel(output_channel)
{
  /* Handles the projection of LiDAR data into a 2D image plane and segmentation of the ground and obstacles.
  
   Attributes
   ----------
   subLaserCloud : Subscription to raw LiDAR point cloud data.
   pubFullCloud : Publisher for the full point cloud projected into a 2D image plane.
   pubFullInfoCloud : Publisher for the full point cloud with additional information per point.
   pubGroundCloud : Publisher for the detected ground points in the point cloud.
   pubSegmentedCloud : Publisher for the segmented non-ground points in the point cloud.
   pubSegmentedCloudPure : Publisher for the pure segmented non-ground points without additional information.
   pubSegmentedCloudInfo : Publisher for metadata information about the segmented cloud.
   pubOutlierCloud : Publisher for outlier points detected during segmentation.
  
   Methods
   -------
   ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel) :
       Constructs an ImageProjection object and initializes ROS communication.
       Parameters include the name of the node and a reference to the output channel for processed data.
  
   void resetParameters() :
       Resets and initializes parameters and storage containers used in LiDAR data processing.
  
   void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) :
       Callback function for the LiDAR point cloud subscription. Handles incoming raw point cloud data.
  
   void projectPointCloud() :
       Projects the raw 3D LiDAR point cloud into a 2D image plane based on the LiDAR's intrinsic parameters.
  
   void findStartEndAngle() :
       Calculates the start and end angles of the LiDAR scan to assist in segmenting the point cloud.
  
   void groundRemoval() :
       Segments the ground from the point cloud based on the sensor's mounting angle and the ground's expected slope.
  
   void cloudSegmentation() :
       Segments the non-ground points for further processing, such as object detection and avoidance.
  
   void labelComponents(int row, int col) :
       Labels the components in the segmented cloud to differentiate between distinct objects.
  
   void publishClouds() :
       Publishes various forms of processed point clouds for visualization and further processing.
 */
  
  subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", 1, std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));

  pubFullCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_projected", 1);
  pubFullInfoCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_info", 1);
  pubGroundCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 1);
  pubSegmentedCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud", 1);
  pubSegmentedCloudPure = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud_pure", 1);
  pubSegmentedCloudInfo = this->create_publisher<cloud_msgs::msg::CloudInfo>("/segmented_cloud_info", 1);
  pubOutlierCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/outlier_cloud", 1);

  // Declare parameters
  this->declare_parameter(PARAM_VERTICAL_SCANS, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_HORIZONTAL_SCANS, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_ANGLE_BOTTOM, rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(PARAM_ANGLE_TOP, rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(PARAM_GROUND_INDEX, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_SENSOR_ANGLE, rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(PARAM_SEGMENT_THETA, rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(PARAM_SEGMENT_POINT, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_SEGMENT_LINE, rclcpp::PARAMETER_INTEGER);

  float vertical_angle_top;

  // Read parameters
  if (!this->get_parameter(PARAM_VERTICAL_SCANS, verticalScans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_VERTICAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_HORIZONTAL_SCANS, horizontalScans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HORIZONTAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_BOTTOM, angBottom)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_BOTTOM.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_TOP, vertical_angle_top)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_TOP.c_str());
  }
  if (!this->get_parameter(PARAM_GROUND_INDEX, groundScanIndex)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s found", PARAM_GROUND_INDEX.c_str());
  }
  if (!this->get_parameter(PARAM_SENSOR_ANGLE, sensorMountAngle)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SENSOR_ANGLE.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_THETA, segmentTheta)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_THETA.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_POINT, segmentValidPointNum)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_POINT.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_LINE, segmentValidLineNum)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_LINE.c_str());
  }

  angResolutionX = (M_PI*2) / (horizontalScans);
  angResolutionY = DEG_TO_RAD*(vertical_angle_top - angBottom) / float(verticalScans-1);
  angBottom = -( angBottom - 0.1) * DEG_TO_RAD;
  segmentTheta *= DEG_TO_RAD;
  sensorMountAngle *= DEG_TO_RAD;

  const size_t cloud_size = verticalScans * horizontalScans;

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);

}

void ImageProjection::resetParameters() {

  /* Summary: Resets and initializes parameters and storage containers for processing a new LiDAR scan.

   Extended Description: Prepares for a new LiDAR scan by resetting internal states, clearing point clouds,
   and reinitializing matrices related to scan processing. It fills point clouds with NaN values to denote 
   unassigned points and sets matrices for range, ground detection, and labeling to their default values. 
   This process ensures that the system is ready for a fresh set of LiDAR data, effectively separating 
   the processing of the current scan from any previous scans.

   Parameters
   ----------
   - cloud_size : size_t
       The total number of points that can be stored in the cloud, calculated based on the LiDAR's vertical and horizontal resolution.
   - nanPoint : PointType
       A point filled with NaN values, used to initialize the point clouds.
   - _laser_cloud_in : pcl::PointCloud<PointType>::Ptr
       The input cloud received from the LiDAR sensor, cleared to remove previous scan data.
   - _ground_cloud : pcl::PointCloud<PointType>::Ptr
       Cloud containing ground points, cleared for the new scan.
   - _segmented_cloud : pcl::PointCloud<PointType>::Ptr
       Cloud containing segmented non-ground points, cleared for the new scan.
   - _segmented_cloud_pure : pcl::PointCloud<PointType>::Ptr
       Cloud containing purely segmented points without ground points, cleared for the new scan.
   - _outlier_cloud : pcl::PointCloud<PointType>::Ptr
       Cloud containing outlier points, cleared for the new scan.
   - _range_mat : Eigen::MatrixXf
       Matrix holding the range (distance) information of each point in the scan, reinitialized to maximum float values.
   - _ground_mat : Eigen::MatrixXi
       Matrix indicating whether a point is ground or not, reset to zero.
   - _label_mat : Eigen::MatrixXi
       Matrix used for labeling points in the cloud, reset to zero.
   - _label_count : int
       Counter for the number of labels used in segmentation, reset to 1.
   - _seg_msg : cloud_msgs::msg::CloudInfo
       Data structure to hold information about the segmented cloud, reset for the new scan processing.

   Returns
   -------
   This function updates the internal state of the ImageProjection object to be ready for the next scan processing.
*/

  const size_t cloud_size = verticalScans * horizontalScans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  _laser_cloud_in->clear();
  _ground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();

  _range_mat.resize(verticalScans, horizontalScans);
  _ground_mat.resize(verticalScans, horizontalScans);
  _label_mat.resize(verticalScans, horizontalScans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);

  _seg_msg.start_ring_index.assign(verticalScans, 0);
  _seg_msg.end_ring_index.assign(verticalScans, 0);

  _seg_msg.segmented_cloud_ground_flag.assign(cloud_size, false);
  _seg_msg.segmented_cloud_col_ind.assign(cloud_size, 0);
  _seg_msg.segmented_cloud_range.assign(cloud_size, 0);
}

void ImageProjection::cloudHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {

  /* Summary: Handles the processing of incoming raw LiDAR point cloud data.

   Extended Description: This method is the main processing pipeline for LiDAR point cloud data. 
   It begins by resetting internal parameters to prepare for a new scan. Then, it converts the incoming
   ROS point cloud message to PCL format, removing any NaN points in the process. It calculates the start
   and end angles of the scan, projects the 3D points into a 2D range image, removes ground points, 
   segments the remaining points into meaningful clusters, and finally publishes the processed data for further use.

   Parameters
   ----------
   - laserCloudMsg : sensor_msgs::msg::PointCloud2::SharedPtr
       The incoming LiDAR scan data in ROS point cloud message format.

   Steps
   -----
   1. Reset internal parameters to prepare for new scan data.
   2. Convert ROS message to PCL point cloud and remove NaN points.
   3. Calculate the start and end angles of the LiDAR scan.
   4. Project the 3D LiDAR points into a 2D range image.
   5. Remove ground points from the projected image.
   6. Segment the non-ground points into clusters.
   7. Publish various processed point clouds for visualization and further processing.

   Returns
   -------
   This function processes the input point cloud and publishes the results.
*/
  // Reset parameters
  resetParameters();

  // Copy and remove NAN points
  pcl::fromROSMsg(*laserCloudMsg, *_laser_cloud_in);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*_laser_cloud_in, *_laser_cloud_in, indices);
  _seg_msg.header = laserCloudMsg->header;

  findStartEndAngle();
  // Range image projection
  projectPointCloud();
  // Mark ground points
  groundRemoval();
  // Point cloud segmentation
  cloudSegmentation();
  //publish (optionally)
  publishClouds();
}


void ImageProjection::projectPointCloud() {

  /* Summary: Projects the 3D LiDAR points into a 2D range image.

   Extended Description: This method takes the 3D point cloud data from the LiDAR and projects it onto a 2D plane to 
   create a range image. Each point's position in the image is determined by its vertical and horizontal angles relative
   to the LiDAR sensor. This projection facilitates the subsequent processing steps, such as ground removal and point cloud segmentation,
   by simplifying the data structure and reducing the computational complexity.

   Parameters
   ----------
   - cloudSize : size_t
       The total number of points in the incoming LiDAR point cloud.
   - thisPoint : PointType
       A temporary variable to store the current point being processed.
   - range : float
       The distance from the LiDAR sensor to the point.
   - verticalAngle : float
       The vertical angle of the point relative to the sensor.
   - rowIdn : int
       The row index in the 2D range image for the current point.
   - horizonAngle : float
       The horizontal angle of the point relative to the sensor.
   - columnIdn : int
       The column index in the 2D range image for the current point.
   - index : size_t
       The index of the point in the flattened 2D range image array.

   Steps
   -----
   1. Calculate the range for each point in the cloud.
   2. Determine the vertical and horizontal angles of each point.
   3. Compute the corresponding row and column indices in the 2D range image.
   4. Store the point in the appropriate location in the 2D range image, along with its range as intensity value.

   Returns
   -------
   This function modifies the internal 2D range image representation of the 3D point cloud.
*/
  // range image projection
  const size_t cloudSize = _laser_cloud_in->points.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];

    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    // find the row and column index in the image for this point
    float verticalAngle = std::asin(thisPoint.z / range);
        //std::atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));

    int rowIdn = (verticalAngle + angBottom) / angResolutionY;
    if (rowIdn < 0 || rowIdn >= verticalScans) {
      continue;
    }

    float horizonAngle = std::atan2(thisPoint.x, thisPoint.y);

    int columnIdn = -round((horizonAngle - M_PI_2) / angResolutionX) + horizontalScans * 0.5;

    if (columnIdn >= horizontalScans){
      columnIdn -= horizontalScans;
    }

    if (columnIdn < 0 || columnIdn >= horizontalScans){
      continue;
    }

    if (range < 0.1){
      continue;
    }

    _range_mat(rowIdn, columnIdn) = range;

    thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

    size_t index = columnIdn + rowIdn * horizontalScans;
    _full_cloud->points[index] = thisPoint;
    // the corresponding range of a point is saved as "intensity"
    _full_info_cloud->points[index] = thisPoint;
    _full_info_cloud->points[index].intensity = range;
  }
}

void ImageProjection::findStartEndAngle() {

  /* Summary: Calculates the start and end orientations of the LiDAR scan.

   Extended Description: This method identifies the starting and ending points of the LiDAR scan based on
   the point cloud data. It calculates the orientations (angles) of the first and last points relative to 
   the LiDAR sensor to determine the overall scan range. These orientations are used in segmenting the point cloud by
   identifying the beginning and end of each scan, which is crucial for accurate segmentation and analysis of the data.

   Parameters
   ----------
   - point : PointType
       A variable to hold the first and last points of the LiDAR scan for calculating orientations.

   Steps
   -----
   1. Calculate the orientation angle for the first point in the point cloud.
   2. Calculate the orientation angle for the last point in the point cloud.
   3. Adjust the end orientation to ensure it represents a complete scan cycle.

   Returns
   -------
   - _seg_msg.start_orientation : float
       The calculated start orientation of the LiDAR scan.
   - _seg_msg.end_orientation : float
       The calculated end orientation of the LiDAR scan, adjusted for continuity.
   - _seg_msg.orientation_diff : float
       The difference in orientation between the start and end of the scan, representing the scan's angular span.

   This function modifies the `start_orientation`, `end_orientation`, and `orientation_diff` fields of the `_seg_msg`.
*/
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();
  _seg_msg.start_orientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();
  _seg_msg.end_orientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  if (_seg_msg.end_orientation - _seg_msg.start_orientation > 3 * M_PI) {
    _seg_msg.end_orientation -= 2 * M_PI;
  } else if (_seg_msg.end_orientation - _seg_msg.start_orientation < M_PI) {
    _seg_msg.end_orientation += 2 * M_PI;
  }
  _seg_msg.orientation_diff =
      _seg_msg.end_orientation - _seg_msg.start_orientation;
}

void ImageProjection::groundRemoval() {

  /* Summary: Segments the ground points from the LiDAR scan.

   Extended Description: This method analyses the point cloud to identify and segment ground points based on the sensor's
   mounting angle and the geometric properties of the points. It utilizes a vertical angle calculation to differentiate 
   ground points from non-ground points. Points are classified into three categories: -1 for no information, 0 for non-ground, and 1 for ground.
   This classification is crucial for subsequent processing stages, such as obstacle detection and path planning.

   Parameters
   ----------
   - lowerInd, upperInd : int
       Indices for the current point and the point directly above it in the point cloud.
   - dX, dY, dZ : float
       Differences in the x, y, and z coordinates between two vertically adjacent points.
   - vertical_angle : float
       The angle between the vertical axis and the line connecting two vertically adjacent points.

   Returns
   -------
   - _ground_mat : Eigen::MatrixXi
       A matrix indicating whether each point is ground (-1 for no info, 0 for non-ground, 1 for ground).
   - _label_mat : Eigen::MatrixXi
       A matrix used for labeling points during segmentation, where ground points and points without valid information are marked as -1.

   This function updates `_ground_mat` to reflect the ground segmentation and modifies `_label_mat` accordingly.

   Note
   ----
   Ground points are identified based on their vertical angle relative to the sensor mount angle. Points with a vertical angle 
   close to the sensor mount angle are considered ground.
*/
  // _ground_mat
  // -1, no valid info to check if ground of not
  //  0, initial value, after validation, means not ground
  //  1, ground
  for (int j = 0; j < horizontalScans; ++j) {
    for (int i = 0; i < groundScanIndex; ++i) {
      int lowerInd = j + (i)*horizontalScans;
      int upperInd = j + (i + 1) * horizontalScans;

      if (_full_cloud->points[lowerInd].intensity == -1 ||
          _full_cloud->points[upperInd].intensity == -1) {
        // no info to check, invalid points
        _ground_mat(i, j) = -1;
        continue;
      }

      float dX =
          _full_cloud->points[upperInd].x - _full_cloud->points[lowerInd].x;
      float dY =
          _full_cloud->points[upperInd].y - _full_cloud->points[lowerInd].y;
      float dZ =
          _full_cloud->points[upperInd].z - _full_cloud->points[lowerInd].z;

      float vertical_angle = std::atan2(dZ , sqrt(dX * dX + dY * dY + dZ * dZ));

      // TODO: review this change

      if ( (vertical_angle - sensorMountAngle) <= 10 * DEG_TO_RAD) {
        _ground_mat(i, j) = 1;
        _ground_mat(i + 1, j) = 1;
      }
    }
  }
  // extract ground cloud (_ground_mat == 1)
  // mark entry that doesn't need to label (ground and invalid point) for
  // segmentation note that ground remove is from 0~_N_scan-1, need _range_mat
  // for mark label matrix for the 16th scan
  for (int i = 0; i < verticalScans; ++i) {
    for (int j = 0; j < horizontalScans; ++j) {
      if (_ground_mat(i, j) == 1 ||
          _range_mat(i, j) == FLT_MAX) {
        _label_mat(i, j) = -1;
      }
    }
  }

  for (int i = 0; i <= groundScanIndex; ++i) {
    for (int j = 0; j < horizontalScans; ++j) {
      if (_ground_mat(i, j) == 1)
        _ground_cloud->push_back(_full_cloud->points[j + i * horizontalScans]);
    }
  }
}

void ImageProjection::cloudSegmentation() {

  /* Summary: Segments the cloud into ground, segmented non-ground, and outlier points.

   Extended Description: This function segments the LiDAR point cloud into ground points, non-ground points, 
   and outliers based on the previously identified ground matrix and labeling. It iterates through the cloud, 
   labeling components and organizing points into their respective categories for further processing.

   Parameters
   ----------
   - Operates directly on class attributes including _ground_mat, _label_mat, and point cloud data.

   Returns
   -------
   - _seg_msg : Contains metadata about the segmentation, including indices and flags for ground points.
   - _segmented_cloud : pcl::PointCloud<PointType>
       Stores the segmented non-ground points.
   - _segmented_cloud_pure : pcl::PointCloud<PointType>
       Stores the pure segmented non-ground points for visualization.
   - _outlier_cloud : pcl::PointCloud<PointType>
       Stores the outlier points detected during segmentation.
   - _label_mat : Eigen::MatrixXi
       Updated during labeling to reflect the segmentation.

   Note
   ----
   The segmentation process identifies points that are not part of the ground and distinguishes between potentially 
   useful non-ground points and outliers. Outliers are points that do not meet the criteria for being included in the segmentation 
   but are not necessarily noise or erroneous readings.
*/
  // segmentation process
  for (int i = 0; i < verticalScans; ++i)
    for (int j = 0; j < horizontalScans; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (int i = 0; i < verticalScans; ++i) {
    _seg_msg.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

    for (int j = 0; j < horizontalScans; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {
          if (i > groundScanIndex && j % 5 == 0) {
            _outlier_cloud->push_back(
                _full_cloud->points[j + i * horizontalScans]);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < horizontalScans - 5) continue;
        }
        // mark ground points so they will not be considered as edge features
        // later
        _seg_msg.segmented_cloud_ground_flag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);
        // mark the points' column index for marking occlusion later
        _seg_msg.segmented_cloud_col_ind[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmented_cloud_range[sizeOfSegCloud] =
            _range_mat(i, j);
        // save seg cloud
        _segmented_cloud->push_back(_full_cloud->points[j + i * horizontalScans]);
        // size of seg cloud
        ++sizeOfSegCloud;
      }
    }

    _seg_msg.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
  }

  // extract segmented cloud for visualization
  for (int i = 0; i < verticalScans; ++i) {
    for (int j = 0; j < horizontalScans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * horizontalScans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

void ImageProjection::labelComponents(int row, int col) {

  /* Summary: Labels connected components in the segmented LiDAR cloud to identify distinct objects.

   Extended Description: This function implements a region growing algorithm to label each point in the segmented cloud. 
   It starts from a seed point and expands to neighboring points based on the similarity criteria determined by the 
   angle threshold `segmentTheta`. Points within this threshold are considered part of the same object and are labeled identically. 
   The algorithm distinguishes between valid segments and outliers, labeling them accordingly for further processing.

   Parameters
   ----------
   - row : int
       The row index of the seed point in the range image.
   - col : int
       The column index of the seed point in the range image.

   - segmentThetaThreshold : float
       Calculated from `segmentTheta`, determines the angular similarity for expanding the segment.
   - lineCountFlag : std::vector<bool>
       Flags to indicate if a row in the range image has been included in the current segment.
   - queue : boost::circular_buffer<Eigen::Vector2i>
       Queue used for the region growing algorithm, stores indices of points to be evaluated.
   - all_pushed : boost::circular_buffer<Eigen::Vector2i>
       Stores all points that have been added to the queue during the segmentation process.
   - neighborIterator : array of Eigen::Vector2i
       Predefined offsets used to iterate over the immediate neighbors of a point in the range image.

   Returns
   -------
   - _label_mat : Eigen::MatrixXi
       Updated with new labels for each point, indicating their membership to a specific segment or marking them as outliers.
   - _label_count : int
       Incremented for each new valid segment identified during the process.

   - _range_mat : Eigen::MatrixXf
       Contains the range (distance) information for each point in the cloud, used to determine point adjacency and similarity.
   - segmentThetaThreshold : float
       The angle threshold used to determine if a neighboring point is part of the same segment.

   Note
   ----
   The function modifies `_label_mat` directly to assign labels to each point, facilitating the segmentation of the cloud into distinct 
   objects based on geometric continuity.
*/

  const float segmentThetaThreshold = tan(segmentTheta);

  std::vector<bool> lineCountFlag(verticalScans, false);
  const size_t cloud_size = verticalScans * horizontalScans;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {
    // Pop point
    Coord2D fromInd = queue.front();
    queue.pop_front();

    // Mark popped point
    _label_mat(fromInd.x(), fromInd.y()) = _label_count;
    // Loop through all the neighboring grids of popped grid

    for (const auto& iter : neighborIterator) {
      // new index
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();
      // index should be within the boundary
      if (thisIndX < 0 || thisIndX >= verticalScans){
        continue;
      }
      // at range image margin (left or right side)
      if (thisIndY < 0){
        thisIndY = horizontalScans - 1;
      }
      if (thisIndY >= horizontalScans){
        thisIndY = 0;
      }
      // prevent infinite loop (caused by put already examined point back)
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha = (iter.x() == 0) ? angResolutionX : angResolutionY;
      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  // check if this segment is valid
  bool feasibleSegment = false;
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }
  else if (static_cast<int>(all_pushed.size()) >= segmentValidPointNum) {
    int lineCount = 0;
    for (int i = 0; i < verticalScans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= segmentValidLineNum) feasibleSegment = true;
  }
  // segment is valid, mark these points
  if (feasibleSegment == true) {
    ++_label_count;
  } else {  // segment is invalid, mark these points
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }
}

void ImageProjection::publishClouds() {

  /* Summary: Publishes various processed point clouds and their associated information.

   Extended Description: This method is responsible for publishing different types of processed LiDAR point clouds, 
   including the full cloud, ground points, segmented cloud, outliers, and more. It checks for subscribers before publishing
   to avoid unnecessary computations. The function also prepares and sends processed data to the next stage in the pipeline 
   through the `_output_channel`.

   Parameters
   ----------
   temp : sensor_msgs::msg::PointCloud2
       A temporary ROS message used for publishing each processed cloud. It is updated with the current header information.
   PublishCloud : Lambda Function
       A lambda function defined within `publishClouds` to streamline the publishing process for different point clouds. 
       It takes a publisher object, a ROS message template, and a point cloud to publish.

   _output_channel : Channel for sending processed output to the next stage.

   Returns
   -------
   - out : ProjectionOut
       An object containing pointers to the outlier cloud, segmented cloud, and the segmented cloud message. 
       This object is sent through the `_output_channel`.

   Publishers
   ----------
   - pubOutlierCloud : Publisher for outlier cloud points.
   - pubSegmentedCloud : Publisher for segmented cloud points.
   - pubFullCloud : Publisher for the complete projected cloud.
   - pubGroundCloud : Publisher for ground points.
   - pubSegmentedCloudPure : Publisher for purely segmented cloud points.
   - pubFullInfoCloud : Publisher for the full cloud with additional info.
   - pubSegmentedCloudInfo : Publisher for segmented cloud metadata.

   Note
   ----
   This function encapsulates the publishing logic for various point clouds and encapsulates the process of preparing data
   for subsequent processing stages.
*/

  sensor_msgs::msg::PointCloud2 temp;
  temp.header.stamp = _seg_msg.header.stamp;
  temp.header.frame_id = "base_link";

  auto PublishCloud = [](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, sensor_msgs::msg::PointCloud2& temp,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (pub->get_subscription_count() != 0) {
      sensor_msgs::msg::PointCloud2 _temp;
      pcl::toROSMsg(*cloud, _temp);
      _temp.header.stamp = temp.header.stamp;
      _temp.header.frame_id = temp.header.frame_id;
      pub->publish(_temp);
    }
  };

  PublishCloud(pubOutlierCloud, temp, _outlier_cloud);
  PublishCloud(pubSegmentedCloud, temp, _segmented_cloud);
  PublishCloud(pubFullCloud, temp, _full_cloud);
  PublishCloud(pubGroundCloud, temp, _ground_cloud);
  PublishCloud(pubSegmentedCloudPure, temp, _segmented_cloud_pure);
  PublishCloud(pubFullInfoCloud, temp, _full_info_cloud);

  if (pubSegmentedCloudInfo->get_subscription_count() != 0) {
    pubSegmentedCloudInfo->publish(_seg_msg);
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());

  std::swap( out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);

  _output_channel.send( std::move(out) );

}
