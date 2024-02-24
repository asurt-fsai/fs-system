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
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "featureAssociation.h"

// Parameters Initializations
const std::string PARAM_VERTICAL_SCANS = "laser.num_vertical_scans";
const std::string PARAM_HORIZONTAL_SCANS = "laser.num_horizontal_scans";
const std::string PARAM_SCAN_PERIOD = "laser.scan_period";
const std::string PARAM_FREQ_DIVIDER = "mapping.mapping_frequency_divider";
const std::string PARAM_EDGE_THRESHOLD = "featureAssociation.edge_threshold";
const std::string PARAM_SURF_THRESHOLD = "featureAssociation.surf_threshold";
const std::string PARAM_DISTANCE = "featureAssociation.nearest_feature_search_distance";

const float RAD2DEG = 180.0 / M_PI;

FeatureAssociation::FeatureAssociation(const std::string &name, Channel<ProjectionOut> &input_channel,
                                       Channel<AssociationOut> &output_channel)
    : Node(name), _input_channel(input_channel), _output_channel(output_channel) {

/* A class for associating features in LiDAR data for SLAM.
 
   Attributes
   ----------
   - pubCornerPointsSharp : Publisher for sharp corner points.
   - pubCornerPointsLessSharp : Publisher for less sharp corner points.
   - pubSurfPointsFlat : Publisher for flat surface points.
   - pubSurfPointsLessFlat : Publisher for less flat surface points.
   - _pub_cloud_corner_last : Publisher for last corner points cloud.
   - _pub_cloud_surf_last : Publisher for last surface points cloud.
   - _pub_outlier_cloudLast : Publisher for last outlier points cloud.
   - pubLaserOdometry : Publisher for laser odometry.
   - tfBroadcaster : Broadcaster for transform.
   - _cycle_count : (int), Cycle counter for controlling the frequency of mapping.
   - _nearest_feature_dist_sqr : (float), Squared distance for nearest feature search.
  
   Methods
   -------
   FeatureAssociation(name, input_channel, output_channel):
       Constructor, initializes the feature association node.
       Parameters:
           - name : std::string
                The name of the node.
           - input_channel : Channel<ProjectionOut>&
                Input channel for projection data.
           - output_channel : Channel<AssociationOut>&
                Output channel for association data.
   ~FeatureAssociation():
       Destructor, cleans up resources.
   initializationValue():
       Initializes internal variable values.
   runFeatureAssociation():
       Main loop for processing feature association.
 */

  pubCornerPointsSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_sharp", 1);
  pubCornerPointsLessSharp = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_sharp", 1);
  pubSurfPointsFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_flat", 1);
  pubSurfPointsLessFlat = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_less_flat", 1);

  _pub_cloud_corner_last = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_corner_last", 2);
  _pub_cloud_surf_last = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf_last", 2);
  _pub_outlier_cloudLast = this->create_publisher<sensor_msgs::msg::PointCloud2>("/outlier_cloud_last", 2);
  pubLaserOdometry = this->create_publisher<nav_msgs::msg::Odometry>("/laser_odom_to_init", 5);

  tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  _cycle_count = 0;

  // Declare parameters
  this->declare_parameter(PARAM_VERTICAL_SCANS, 32);
  this->declare_parameter(PARAM_HORIZONTAL_SCANS, 1800);
  this->declare_parameter(PARAM_SCAN_PERIOD, 0.1);
  this->declare_parameter(PARAM_FREQ_DIVIDER, 5);
  this->declare_parameter(PARAM_EDGE_THRESHOLD, 0.1);
  this->declare_parameter(PARAM_SURF_THRESHOLD, 0.1);
  this->declare_parameter(PARAM_DISTANCE, 5.0);

  float nearest_dist;

  // Read parameters
  if (!this->get_parameter(PARAM_VERTICAL_SCANS, _vertical_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_VERTICAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_HORIZONTAL_SCANS, _horizontal_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HORIZONTAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_SCAN_PERIOD, _scan_period)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SCAN_PERIOD.c_str());
  }
  if (!this->get_parameter(PARAM_FREQ_DIVIDER, _mapping_frequency_div)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_FREQ_DIVIDER.c_str());
  }
  if (!this->get_parameter(PARAM_EDGE_THRESHOLD, _edge_threshold)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_EDGE_THRESHOLD.c_str());
  }
  if (!this->get_parameter(PARAM_SURF_THRESHOLD, _surf_threshold)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SURF_THRESHOLD.c_str());
  }
  if (!this->get_parameter(PARAM_DISTANCE, nearest_dist)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_DISTANCE.c_str());
  }

  _nearest_feature_dist_sqr = nearest_dist*nearest_dist;

  initializationValue();

 _run_thread = std::thread (&FeatureAssociation::runFeatureAssociation, this);
}

FeatureAssociation::~FeatureAssociation()
{
  /* Destructor for FeatureAssociation.

   This method ensures that the feature association process is properly terminated by 
   signaling the input channel to stop and waiting for the processing thread to finish.

   Attributes
   ----------
   _input_channel : Channel<ProjectionOut>
       The channel used to receive projection data for feature association.
   _run_thread : std::thread
       The thread on which the feature association process runs.

   Methods
   -------
   ~FeatureAssociation():
       Sends an empty message to the input channel, signaling it to stop, and joins 
       the run thread to ensure all processing has completed before the object is destroyed.
*/

  _input_channel.send({});
  _run_thread.join();
}

void FeatureAssociation::initializationValue() {

  /* Summary: Initializes internal variables and data structures for feature association.
  
     Extended Description: This method prepares the necessary initial conditions for the feature association process by 
     setting up point clouds and internal data structures. It configures the environment to ensure accurate and efficient 
     processing of each new lidar scan, establishing a foundation for subsequent feature extraction, association, and odometry updates.

     Parameters (initialized within function)
     ----------
     - cloud_size : size_t
         The total number of points expected in the cloud, calculated from the number of vertical and horizontal scans. 
         Dictates the size of various vectors used throughout the process.
     - downSizeFilter : pcl::VoxelGrid<PointType>
         A voxel grid filter configured to downsample point clouds to a specified resolution, enhancing processing efficiency.
     - segmentedCloud, outlierCloud, cornerPointsSharp, cornerPointsLessSharp, surfPointsFlat, surfPointsLessFlat,
       surfPointsLessFlatScan, surfPointsLessFlatScanDS : pcl::PointCloud<PointType>::Ptr
         Point clouds initialized to store different types of features extracted from the raw lidar data, each serving 
         a specific role in the feature association and mapping process.
     - cloudCurvature, cloudNeighborPicked, cloudLabel : std::vector<float>
         Vectors resized according to `cloud_size`, used to store properties of each point in the cloud such as curvature 
         and flags for processing status.
     - pointSearchCornerInd1, pointSearchCornerInd2, pointSearchSurfInd1, pointSearchSurfInd2, pointSearchSurfInd3 : std::vector<int>
         Vectors resized based on `cloud_size`, utilized for indexing points within the point clouds during the feature association steps.
     - skipFrameNum : int
         Used to control the frequency of processing frames, affecting the temporal resolution of feature association.
     - transformCur, transformSum : std::array<float, 6>
         Arrays holding the current and cumulative transformation parameters, essential for calculating odometry and aligning point clouds.
     - systemInitedLM : bool
         Indicates whether the system initialization process has been completed, used to trigger the start of normal processing routines.
     - isDegenerate : bool
         Indicates if the current state of the system is degenerate, which necessitates special handling during optimization steps.
     - frameCount : int
         A counter used to manage the frequency of certain operations, like publishing processed point clouds, based on `skipFrameNum`.

     Returns
     -------
     The function initializes and prepares internal state for lidar data processing, affecting class member variables.
  */

  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  cloudSmoothness.resize(cloud_size);

  downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

  segmentedCloud.reset(new pcl::PointCloud<PointType>());
  outlierCloud.reset(new pcl::PointCloud<PointType>());

  cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
  cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
  surfPointsFlat.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

  surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
  surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

  cloudCurvature.resize(cloud_size);
  cloudNeighborPicked.resize(cloud_size);
  cloudLabel.resize(cloud_size);

  pointSearchCornerInd1.resize(cloud_size);
  pointSearchCornerInd2.resize(cloud_size);

  pointSearchSurfInd1.resize(cloud_size);
  pointSearchSurfInd2.resize(cloud_size);
  pointSearchSurfInd3.resize(cloud_size);

  skipFrameNum = 1;

  for (int i = 0; i < 6; ++i) {
    transformCur[i] = 0;
    transformSum[i] = 0;
  }

  systemInitedLM = false;

  laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
  laserCloudOri.reset(new pcl::PointCloud<PointType>());
  coeffSel.reset(new pcl::PointCloud<PointType>());

  laserOdometry.header.frame_id = "camera_init";
  laserOdometry.child_frame_id = "laser_odom";

  laserOdometryTrans.header.frame_id = "camera_init";
  laserOdometryTrans.child_frame_id = "laser_odom";

  isDegenerate = false;

  frameCount = skipFrameNum;
}

void FeatureAssociation::adjustDistortion() {

  /* Summary: Corrects motion distortion in segmented lidar points.

     Extended Description: This method iteratively processes each point in the `segmentedCloud`, adjusting its position 
     to correct for distortions caused by the motion of the lidar sensor during the scan. It utilizes the scan's metadata 
     (start and end orientations, relative times of points) stored in `segInfo` to calculate the necessary adjustments. 
     Each point's position and intensity are recalibrated to represent their true location as if captured from a stationary 
     perspective. This adjustment is crucial for accurate mapping and localization by compensating for the lidar's movement.

     Parameters
     ----------
     - halfPassed : bool
         Determines if the scan processing has surpassed the 180-degree mark.
     - cloudSize : int
         Total number of points in `segmentedCloud`, used to iterate through the cloud.
     - point : PointType
         A temporary variable to store the current point being adjusted. It has x, y, z coordinates and intensity.
     - ori : float
         The original orientation angle of the point, calculated based on its position.
     - relTime : float
        Relative time offset of the point within the scan, used to adjust the point's intensity.

     Returns
     -------
     The primary output of this function is the in-place modification of `segmentedCloud`, 
     where each point is adjusted to correct for any motion distortion caused by the lidar's movement during the scan.
  */

  bool halfPassed = false;
  int cloudSize = segmentedCloud->points.size();

  PointType point;

  for (int i = 0; i < cloudSize; i++) {
    point.x = segmentedCloud->points[i].y;
    point.y = segmentedCloud->points[i].z;
    point.z = segmentedCloud->points[i].x;

    float ori = -atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < segInfo.start_orientation - M_PI / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.start_orientation + M_PI * 3 / 2)
        ori -= 2 * M_PI;

      if (ori - segInfo.start_orientation > M_PI) halfPassed = true;
    } else {
      ori += 2 * M_PI;

      if (ori < segInfo.end_orientation - M_PI * 3 / 2)
        ori += 2 * M_PI;
      else if (ori > segInfo.end_orientation + M_PI / 2)
        ori -= 2 * M_PI;
    }

    float relTime = (ori - segInfo.start_orientation) / segInfo.orientation_diff;
    point.intensity =
        int(segmentedCloud->points[i].intensity) + _scan_period * relTime;

    segmentedCloud->points[i] = point;
  }
}

void FeatureAssociation::calculateSmoothness() {

  /* Summary: Calculates the smoothness of each point in the segmented cloud.
  
     Extended Description: This method iterates through the points in the `segmentedCloud` and calculates the smoothness
     for each point based on the difference in range values with its neighbors. The smoothness value is used to 
     differentiate between flat surfaces and edges or corners in the environment. Points are then marked as not picked 
     and initially unlabelled, preparing them for subsequent feature extraction processes.

     Parameters
     ----------
     cloudSize : int
         The total number of points in the segmented cloud, used to iterate through the cloud.
     diffRange : float
         The calculated difference in range between a point and its immediate neighbors, used to calculate the point's smoothness.
     cloudCurvature : std::vector<float>
         A vector that stores the calculated curvature (smoothness) for each point. It is directly modified to record the smoothness values.
     cloudNeighborPicked : std::vector<int>
         A vector indicating whether a point has been picked as a feature or neighbor. It is reset for all points in this context.
     cloudLabel : std::vector<int>
         A vector to store the label of each point (unlabelled, edge, or surface). Initially set to 0 (unlabelled) for all points.
     cloudSmoothness : std::vector<pair<float, int>>
         A vector of pairs, where each pair contains the smoothness value and the index of a point. This is used for sorting points
         based on their smoothness.

     Returns
     -------
     The function modifies `cloudCurvature`, `cloudNeighborPicked`, `cloudLabel`, and `cloudSmoothness` in place to 
     reflect the calculated smoothness values and initial labels for each point in the segmented cloud.
  */

  int cloudSize = segmentedCloud->points.size();
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffRange = segInfo.segmented_cloud_range[i - 5] +
                      segInfo.segmented_cloud_range[i - 4] +
                      segInfo.segmented_cloud_range[i - 3] +
                      segInfo.segmented_cloud_range[i - 2] +
                      segInfo.segmented_cloud_range[i - 1] -
                      segInfo.segmented_cloud_range[i] * 10 +
                      segInfo.segmented_cloud_range[i + 1] +
                      segInfo.segmented_cloud_range[i + 2] +
                      segInfo.segmented_cloud_range[i + 3] +
                      segInfo.segmented_cloud_range[i + 4] +
                      segInfo.segmented_cloud_range[i + 5];

    cloudCurvature[i] = diffRange * diffRange;

    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    cloudSmoothness[i].value = cloudCurvature[i];
    cloudSmoothness[i].ind = i;
  }
}

void FeatureAssociation::markOccludedPoints() {

  /* Summary: Identifies and marks occluded points and points with significant range differences in the segmented cloud.
  
     Extended Description: This function examines the depth differences between consecutive points in the `segmentedCloud`
      to identify occluded points or points that are likely to be on the edge of objects, where occlusion or significant depth
       changes occur. Points identified as occluded or having significant depth changes relative to their neighbors are marked
        in the `cloudNeighborPicked` vector. This marking process is crucial for avoiding the selection of unreliable points 
        during feature extraction, especially for edge or corner features.

     Parameters
     ----------
     cloudSize : int
         The total number of points in the segmented cloud, used for iteration.
     depth1, depth2 : float
         The depth values of consecutive points in the cloud, used to calculate depth differences.
     columnDiff : int
         The difference in column indices between consecutive points, indicating their relative position within the scan.
     diff1, diff2 : float
         The absolute difference in depth between a point and its immediate neighbors, used to identify significant depth changes.

     Returns
     -------
     - cloudNeighborPicked : std::vector<int>
         A vector that indicates whether a point has been marked as occluded or having a significant depth change. Points identified
          as such are marked by setting their corresponding indices to 1.

     The function directly modifies the `cloudNeighborPicked` vector to mark occluded points and points with significant depth changes.
  */

  int cloudSize = segmentedCloud->points.size();

  for (int i = 5; i < cloudSize - 6; ++i) {
    float depth1 = segInfo.segmented_cloud_range[i];
    float depth2 = segInfo.segmented_cloud_range[i + 1];
    int columnDiff = std::abs(int(segInfo.segmented_cloud_col_ind[i + 1] -
                                  segInfo.segmented_cloud_col_ind[i]));

    if (columnDiff < 10) {
      if (depth1 - depth2 > 0.3) {
        cloudNeighborPicked[i - 5] = 1;
        cloudNeighborPicked[i - 4] = 1;
        cloudNeighborPicked[i - 3] = 1;
        cloudNeighborPicked[i - 2] = 1;
        cloudNeighborPicked[i - 1] = 1;
        cloudNeighborPicked[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        cloudNeighborPicked[i + 1] = 1;
        cloudNeighborPicked[i + 2] = 1;
        cloudNeighborPicked[i + 3] = 1;
        cloudNeighborPicked[i + 4] = 1;
        cloudNeighborPicked[i + 5] = 1;
        cloudNeighborPicked[i + 6] = 1;
      }
    }

    float diff1 = std::abs(float(segInfo.segmented_cloud_range[i-1] - segInfo.segmented_cloud_range[i]));
    float diff2 = std::abs(float(segInfo.segmented_cloud_range[i+1] - segInfo.segmented_cloud_range[i]));

    if (diff1 > 0.02 * segInfo.segmented_cloud_range[i] &&
        diff2 > 0.02 * segInfo.segmented_cloud_range[i])
      cloudNeighborPicked[i] = 1;
  }
}

void FeatureAssociation::extractFeatures() {

  /* Summary: Extracts sharp and less sharp corner points, flat and less flat surface points from the segmented cloud.
  
     Extended Description: This function processes each scan line of the segmented lidar cloud to identify and categorize 
     points based on their curvature and ground flag. It sorts points within each scan segment based on their smoothness 
     (curvature) and selects sharp corner points, less sharp corner points, flat surface points, and less flat surface points
     based on predefined thresholds. This categorization is crucial for subsequent steps in the SLAM process, such as feature 
     matching and map optimization.

     Parameters
     ----------
     _vertical_scans : int
         The number of vertical scan lines in the lidar data, used to iterate over each scan line.
     sp, ep : int
         Start and end points for processing segments within a scan line, calculated for each segment.
     largestPickedNum, smallestPickedNum : int
         Counters for the number of points picked as sharp or flat within each segment.
     ind : int
         Index of the current point being evaluated within the cloudSmoothness vector.
     cloudSmoothness : std::vector<pair<float, int>>
         A vector containing pairs of smoothness values and corresponding point indices, sorted to facilitate feature extraction.
     _edge_threshold, _surf_threshold : float
         Thresholds for distinguishing between sharp edges, less sharp edges, and flat surfaces based on point curvature.

     Returns
     -------
     - cornerPointsSharp, cornerPointsLessSharp : pcl::PointCloud<PointType>::Ptr
         Point clouds that are filled with sharp and less sharp corner points extracted from the segmented cloud.
     - surfPointsFlat, surfPointsLessFlat : pcl::PointCloud<PointType>::Ptr
         Point clouds that are filled with flat and less flat surface points extracted from the segmented cloud.
     - cloudNeighborPicked : std::vector<int>
         Marks points that have been picked to avoid their reselection.
     - cloudLabel : std::vector<int>
         Labels points based on their selection as corner or surface features.

     This function modifies the point clouds `cornerPointsSharp`, `cornerPointsLessSharp`, `surfPointsFlat`, and 
     `surfPointsLessFlat` in place, filling them with the respective types of features extracted from the segmented cloud.
  */

  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  for (int i = 0; i < _vertical_scans; i++) {
    surfPointsLessFlatScan->clear();

    for (int j = 0; j < 6; j++) {
      int sp =
          (segInfo.start_ring_index[i] * (6 - j) + segInfo.end_ring_index[i] * j) /
          6;
      int ep = (segInfo.start_ring_index[i] * (5 - j) +
                segInfo.end_ring_index[i] * (j + 1)) /
                   6 -
               1;

      if (sp >= ep) continue;

      std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep,
                by_value());

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > _edge_threshold &&
            segInfo.segmented_cloud_ground_flag[ind] == false) {
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(segmentedCloud->points[ind]);
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(segmentedCloud->points[ind]);
          } else {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                             segInfo.segmented_cloud_col_ind[ind + l - 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if( ind + l < 0 ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind[ind + l] -
                             segInfo.segmented_cloud_col_ind[ind + l + 1]));
            if (columnDiff > 10) break;
            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSmoothness[k].ind;
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < _surf_threshold &&
            segInfo.segmented_cloud_ground_flag[ind] == true) {
          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(segmentedCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            if ( ind + l >= static_cast<int>(segInfo.segmented_cloud_col_ind.size()) ) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                             segInfo.segmented_cloud_col_ind.at(ind + l - 1)));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            if (ind + l < 0) {
              continue;
            }
            int columnDiff =
                std::abs(int(segInfo.segmented_cloud_col_ind.at(ind + l) -
                             segInfo.segmented_cloud_col_ind.at(ind + l + 1)));
            if (columnDiff > 10) break;

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(segmentedCloud->points[k]);
        }
      }
    }

    surfPointsLessFlatScanDS->clear();
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);

    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }
}

void FeatureAssociation::TransformToStart(PointType const *const pi,
                                          PointType *const po) {
  
 /* Summary: Transforms a point from the end of the scan to the start of the scan.
 
    Extended Description: This function adjusts a point's position based on its intensity value, which encodes
    the relative time within the scan period. It uses the current state of the transformation
    (transformCur) to calculate the adjusted position. The adjustment compensates for the motion
    of the LiDAR sensor through the environment, improving the alignment of point cloud data
    collected at different times during the scan.
  
    Parameters
    ----------
    pi : PointType const *const
         The input point to be transformed. The intensity field of the point should
         encode the relative time at which the point was measured.
    po : PointType *const
         The output point after transformation. This point's coordinates are adjusted
         to account for sensor motion, effectively "rewinding" its position to the
         start of the scan.

    Returns
    -------
    po : PointType *const
         The function modifies the coordinates and intensity of the output point `po`,
         aligning it with the start of the scan period based on the sensor's motion.
  
    Note
    ----
    The function directly modifies the coordinates and intensity of the output point `po`,
    but leaves its other attributes unchanged. The transformation is based on the fractional
    part of the point's intensity, which is assumed to represent the relative time offset
    within the scan period.
 */

  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->intensity = pi->intensity;
}

void FeatureAssociation::TransformToEnd(PointType const *const pi,
                                        PointType *const po) {

  /* Summary: Transforms a point to its position at the end of the scan.
     
     Extended Description: This function applies a transformation to a point based on its intensity, which encodes
     the relative time within the scan, adjusting its position to where it would be at the
     end of the scanning period. It takes into account the motion of the LiDAR sensor and the
     current transformation state (transformCur) to perform this adjustment. This helps in aligning
     points collected at different times into a consistent frame of reference corresponding to the
     end of the scan period.

     Parameters
     ----------
     pi : PointType const *const
          The input point to be transformed. The intensity field of the point encodes
          the relative time at which the point was measured, used for adjusting its position.
     po : PointType *const
          The output point after transformation. This is where the adjusted position of the input
          point is stored, representing its estimated location at the end of the scan.
  
     Returns
     --------
     po : PointType *const
          Directly modifies the `po` parameter, setting its coordinates and intensity to reflect
          the point's transformed position at the end of the scan period.
 
     Note
     ----
     The function computes the transformation by considering the fractional part of the point's
     intensity as a time offset within the scan period. The final position is calculated using
     the current sensor motion state (transformCur). The function operates by modifying the `po`
     parameter directly and does not return a value.
  */
  float s = 10 * (pi->intensity - int(pi->intensity));

  float rx = s * transformCur[0];
  float ry = s * transformCur[1];
  float rz = s * transformCur[2];
  float tx = s * transformCur[3];
  float ty = s * transformCur[4];
  float tz = s * transformCur[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transformCur[0];
  ry = transformCur[1];
  rz = transformCur[2];
  tx = transformCur[3];
  ty = transformCur[4];
  tz = transformCur[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  po->x = x6;
  po->y = y6;
  po->z = z6;
  po->intensity = int(pi->intensity);
}


void FeatureAssociation::AccumulateRotation(float cx, float cy, float cz,
                                            float lx, float ly, float lz,
                                            float &ox, float &oy, float &oz) {

  /* Summary: Accumulates rotation from a current and a previous orientation.

     Extended Description: This function calculates the overall rotation by accumulating the difference between the
     current orientation (cx, cy, cz) and the last orientation (lx, ly, lz). It applies the
     rotation accumulation in a way that accounts for the non-commutativity of rotation operations,
     ensuring that the sequence of rotations is correctly applied to yield the final orientation.
     The function updates the output orientation parameters (ox, oy, oz) to reflect the accumulated
     rotation. 

     Parameters
     ----------
     cx : float
          Current rotation about the x-axis in radians.
     cy : float
          Current rotation about the y-axis in radians.
     cz : float
          Current rotation about the z-axis in radians.
     lx : float
          Last rotation about the x-axis in radians.
     ly : float
          Last rotation about the y-axis in radians.
     lz : float
          Last rotation about the z-axis in radians.
     ox : float&
          Output rotation about the x-axis in radians, after accumulation.
     oy : float&
          Output rotation about the y-axis in radians, after accumulation.
     oz : float&
          Output rotation about the z-axis in radians, after accumulation.
 
     Returns
     -------
     Modified ox, oy, oz parameters to contain the result of the accumulated
     rotation, representing the final orientation after applying the current rotation on top of
     the last rotation.
  
    Note
    ----
    This function is used to maintain an accurate representation of the sensor's orientation over
    time, adjusting for new rotation measurements. It is essential for tasks that require precise
    tracking of orientation changes, such as in navigation and 3D mapping applications.
 */

  float srx = cos(lx) * cos(cx) * sin(ly) * sin(cz) -
              cos(cx) * cos(cz) * sin(lx) - cos(lx) * cos(ly) * sin(cx);
  ox = -asin(srx);

  float srycrx =
      sin(lx) * (cos(cy) * sin(cz) - cos(cz) * sin(cx) * sin(cy)) +
      cos(lx) * sin(ly) * (cos(cy) * cos(cz) + sin(cx) * sin(cy) * sin(cz)) +
      cos(lx) * cos(ly) * cos(cx) * sin(cy);
  float crycrx =
      cos(lx) * cos(ly) * cos(cx) * cos(cy) -
      cos(lx) * sin(ly) * (cos(cz) * sin(cy) - cos(cy) * sin(cx) * sin(cz)) -
      sin(lx) * (sin(cy) * sin(cz) + cos(cy) * cos(cz) * sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx =
      sin(cx) * (cos(lz) * sin(ly) - cos(ly) * sin(lx) * sin(lz)) +
      cos(cx) * sin(cz) * (cos(ly) * cos(lz) + sin(lx) * sin(ly) * sin(lz)) +
      cos(lx) * cos(cx) * cos(cz) * sin(lz);
  float crzcrx =
      cos(lx) * cos(lz) * cos(cx) * cos(cz) -
      cos(cx) * sin(cz) * (cos(ly) * sin(lz) - cos(lz) * sin(lx) * sin(ly)) -
      sin(cx) * (sin(ly) * sin(lz) + cos(ly) * cos(lz) * sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void FeatureAssociation::findCorrespondingCornerFeatures(int iterCount) {

  /* Summary: Identifies corresponding corner features in the latest corner point cloud.

     Extended Description: This function searches for the closest corner feature points in the previously
     stored corner point cloud for each point in the current sharp corner points set.
     It uses the k-d tree to find the nearest neighbors and computes the feature
     relationship for matching. The results are used for further optimization processes.

     Parameters
     ----------
     iterCount : int
       The current iteration count in the feature association process. This parameter
       controls the frequency of executing certain parts of the function logic, optimizing
       performance by reducing the number of nearest neighbor searches.
 
     Returns
     -------
     Updates `laserCloudOri` and `coeffSel` with points and coefficients for feature constraints.
     These are used in optimization to adjust the pose estimation.
    - `pointSearchCornerInd1` and `pointSearchCornerInd2` by storing indices of the found points.
    - `laserCloudOri` by adding selected corner points for optimization.
    - `coeffSel` by adding coefficients based on the geometric relation of the corner points for optimization.
  */

  int cornerPointsSharpNum = cornerPointsSharp->points.size();

  for (int i = 0; i < cornerPointsSharpNum; i++) {
    PointType pointSel;
    TransformToStart(&cornerPointsSharp->points[i], &pointSel);

    if (iterCount % 5 == 0) {
      kdtreeCornerLast.nearestKSearch(pointSel, 1, pointSearchInd,
                                       pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1;

      if (pointSearchSqDis[0] < _nearest_feature_dist_sqr) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan =
            int(laserCloudCornerLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = _nearest_feature_dist_sqr;
        for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
          if (int(laserCloudCornerLast->points[j].intensity) >
              closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) >
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudCornerLast->points[j].intensity) <
              closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) *
                           (laserCloudCornerLast->points[j].x - pointSel.x) +
                       (laserCloudCornerLast->points[j].y - pointSel.y) *
                           (laserCloudCornerLast->points[j].y - pointSel.y) +
                       (laserCloudCornerLast->points[j].z - pointSel.z) *
                           (laserCloudCornerLast->points[j].z - pointSel.z);

          if (int(laserCloudCornerLast->points[j].intensity) <
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
      }

      pointSearchCornerInd1[i] = closestPointInd;
      pointSearchCornerInd2[i] = minPointInd2;
    }

    if (pointSearchCornerInd2[i] >= 0) {
      PointType tripod1 =
          laserCloudCornerLast->points[pointSearchCornerInd1[i]];
      PointType tripod2 =
          laserCloudCornerLast->points[pointSearchCornerInd2[i]];

      float x0 = pointSel.x;
      float y0 = pointSel.y;
      float z0 = pointSel.z;
      float x1 = tripod1.x;
      float y1 = tripod1.y;
      float z1 = tripod1.z;
      float x2 = tripod2.x;
      float y2 = tripod2.y;
      float z2 = tripod2.z;

      float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
      float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
      float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

      float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

      float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                       (z1 - z2) * (z1 - z2));

      float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;

      float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;

      float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;

      float ld2 = a012 / l12;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 - 1.8 * fabs(ld2);
      }

      if (s > 0.1 && ld2 != 0) {
        PointType coeff;
        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        laserCloudOri->push_back(cornerPointsSharp->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

void FeatureAssociation::findCorrespondingSurfFeatures(int iterCount) {

  /* Summary: Searches for corresponding flat surface features in the latest surface point cloud.

     Extended Description: This function aims to find the best matching surface features in the previous
     surface point cloud for each point in the current set of flat surface points.
     It leverages a k-d tree for efficient nearest neighbor searches to identify
     points that are potentially part of the same surface. These correspondences
     are then used to establish constraints for pose optimization.

     Parameters
     ----------
     - iterCount (int): The iteration count within the feature association cycle, 
     used to optimize computational performance by executing the search selectively.

     Returns
     -------
     Updates `pointSearchSurfInd1`, `pointSearchSurfInd2`, `pointSearchSurfInd3` with indices 
     of matched points in the surface point cloud.
     Appends to `laserCloudOri` with points that are part of detected surfaces.
     Adds to `coeffSel` with coefficients representing the geometric relationship between matched 
     surface points for optimization.
  */
  int surfPointsFlatNum = surfPointsFlat->points.size();

  for (int i = 0; i < surfPointsFlatNum; i++) {
    PointType pointSel;
    TransformToStart(&surfPointsFlat->points[i], &pointSel);

    if (iterCount % 5 == 0) {
      kdtreeSurfLast.nearestKSearch(pointSel, 1, pointSearchInd,
                                     pointSearchSqDis);
      int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

      if (pointSearchSqDis[0] < _nearest_feature_dist_sqr) {
        closestPointInd = pointSearchInd[0];
        int closestPointScan =
            int(laserCloudSurfLast->points[closestPointInd].intensity);

        float pointSqDis, minPointSqDis2 = _nearest_feature_dist_sqr,
                          minPointSqDis3 = _nearest_feature_dist_sqr;
        for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
          if (int(laserCloudSurfLast->points[j].intensity) >
              closestPointScan + 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) <=
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
        for (int j = closestPointInd - 1; j >= 0; j--) {
          if (int(laserCloudSurfLast->points[j].intensity) <
              closestPointScan - 2.5) {
            break;
          }

          pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) *
                           (laserCloudSurfLast->points[j].x - pointSel.x) +
                       (laserCloudSurfLast->points[j].y - pointSel.y) *
                           (laserCloudSurfLast->points[j].y - pointSel.y) +
                       (laserCloudSurfLast->points[j].z - pointSel.z) *
                           (laserCloudSurfLast->points[j].z - pointSel.z);

          if (int(laserCloudSurfLast->points[j].intensity) >=
              closestPointScan) {
            if (pointSqDis < minPointSqDis2) {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          } else {
            if (pointSqDis < minPointSqDis3) {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }
        }
      }

      pointSearchSurfInd1[i] = closestPointInd;
      pointSearchSurfInd2[i] = minPointInd2;
      pointSearchSurfInd3[i] = minPointInd3;
    }

    if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
      PointType tripod1 = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
      PointType tripod2 = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
      PointType tripod3 = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

      float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) -
                 (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
      float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) -
                 (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
      float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) -
                 (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
      float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

      float ps = sqrt(pa * pa + pb * pb + pc * pc);

      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

      float s = 1;
      if (iterCount >= 5) {
        s = 1 -
            1.8 * fabs(pd2) /
                sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y +
                          pointSel.z * pointSel.z));
      }

      if (s > 0.1 && pd2 != 0) {
        PointType coeff;
        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        laserCloudOri->push_back(surfPointsFlat->points[i]);
        coeffSel->push_back(coeff);
      }
    }
  }
}

bool FeatureAssociation::calculateTransformationSurf(int iterCount) {

/* Summary: Calculates the transformation needed to align the current scan with the last scan.
  
     Extended Description: This function constructs and solves a linear system based on the correspondences 
     between selected points in the current scan and their counterparts in the last scan. It utilizes the 
     least squares method to optimize the transformation parameters, ensuring the best possible alignment. 
     The process is iterative, with early termination if the adjustments fall below a predefined threshold, 
     indicating convergence.

     Parameters
     ----------
     iterCount : int
         The current iteration count within the optimization loop, controlling the adaptiveness of the process.
     pointSelNum : int
         The number of selected points used for the transformation calculation.
     matA, matAt, matAtA : Eigen::Matrix<float, Eigen::Dynamic, 6>, Eigen::Matrix<float, 6, Eigen::Dynamic>, Eigen::Matrix<float, 6, 6>
         Matrices used in constructing the linear system for optimization.
     matB : Eigen::VectorXf
         The vector representing the right-hand side of the linear system.
     matX : Eigen::Matrix<float, 6, 1>
         The solution vector, representing the calculated adjustments to the transformation parameters.
     srx, crx, sry, cry, srz, crz, tx, ty, tz : float
         Sin and cos values of rotations, and translations used in the linear system construction.

     Returns
     -------
     bool : Indicates whether the transformation has converged based on the magnitude of the calculated adjustments.

     - transformCur: Applies the calculated adjustments to the current transformation parameters.
  */
  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,3> matA(pointSelNum, 3);
  Eigen::Matrix<float,3,Eigen::Dynamic> matAt(3,pointSelNum);
  Eigen::Matrix<float,3,3> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,3,1> matAtB;
  Eigen::Matrix<float,3,1> matX;
  Eigen::Matrix<float,3,3> matP;

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx * sry * srz;
  float a2 = crx * crz * sry;
  float a3 = srx * sry;
  float a4 = tx * a1 - ty * a2 - tz * a3;
  float a5 = srx * srz;
  float a6 = crz * srx;
  float a7 = ty * a6 - tz * crx - tx * a5;
  float a8 = crx * cry * srz;
  float a9 = crx * cry * crz;
  float a10 = cry * srx;
  float a11 = tz * a10 + ty * a9 - tx * a8;

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;

  float c1 = -b6;
  float c2 = b5;
  float c3 = tx * b6 - ty * b5;
  float c4 = -crx * crz;
  float c5 = crx * srz;
  float c6 = ty * c5 + tx * -c4;
  float c7 = b2;
  float c8 = -b1;
  float c9 = tx * -b2 - ty * -b1;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float arx =
        (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x +
        (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y +
        (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

    float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x +
                (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y +
                (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = arx;
    matA(i, 1) = arz;
    matA(i, 2) = aty;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1,3> matE;
    Eigen::Matrix<float,3,3> matV;
    Eigen::Matrix<float,3,3> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,3,3> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,3,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[0] += matX(0, 0);
  transformCur[2] += matX(1, 0);
  transformCur[4] += matX(2, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2) +
                      pow(RAD2DEG * (matX(1, 0)), 2));
  float deltaT = sqrt(pow(matX(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

bool FeatureAssociation::calculateTransformationCorner(int iterCount) {

/* Summary: Optimizes corner feature alignment to compute transformation adjustments.
   
   Extended Description: This function iteratively refines the pose estimation by aligning 
   corner features between consecutive scans. It constructs and solves a linear system derived 
   from the geometric relationship of corner features. The process involves selecting corner points, 
   determining their corresponding points in the previous frame using a k-d tree, and computing the 
   optimal rotation and translation adjustments that minimize the alignment error.

   Parameters
   ----------
   - iterCount (int): The current iteration number in the optimization cycle, controlling the 
     adaptiveness of the optimization process.

   Returns
   -------
    bool: Indicates whether the transformation adjustments have converged, based on the magnitude
     of the changes in rotation and translation. Returns true if adjustments are below a predefined
     threshold, suggesting that the pose estimation has stabilized.

   - transformCur: Directly updates the current transformation estimates (rotation and translation)
     based on the optimization results.
   - isDegenerate: Indicates if the optimization problem is degenerate, which can affect the solution 
     stability and require special handling.
   - laserCloudOri, coeffSel: Populated with selected corner points and their corresponding coefficients,
     which are used for pose optimization.
*/

  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,3> matA(pointSelNum, 3);
  Eigen::Matrix<float,3,Eigen::Dynamic> matAt(3,pointSelNum);
  Eigen::Matrix<float,3,3> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,3,1> matAtB;
  Eigen::Matrix<float,3,1> matX;
  Eigen::Matrix<float,3,3> matP;

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b3 = crx * cry;
  float b4 = tx * -b1 + ty * -b2 + tz * b3;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;
  float b7 = crx * sry;
  float b8 = tz * b7 - ty * b6 - tx * b5;

  float c5 = crx * srz;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float ary =
        (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x +
        (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = ary;
    matA(i, 1) = atx;
    matA(i, 2) = atz;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1, 3> matE;
    Eigen::Matrix<float,3, 3> matV;
    Eigen::Matrix<float,3, 3> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,3,3> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[3] = {10, 10, 10};
    for (int i = 2; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 3; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,3,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[1] += matX(0, 0);
  transformCur[3] += matX(1, 0);
  transformCur[5] += matX(2, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2));
  float deltaT = sqrt(pow(matX(1, 0) * 100, 2) +
                      pow(matX(2, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

bool FeatureAssociation::calculateTransformation(int iterCount) {

/* Summary: Computes the overall transformation by optimizing the alignment of both corner and surface features.

   Extended Description: This function iteratively refines the overall transformation between consecutive
    scans by aligning corner and surface features together. It builds a combined optimization problem using
    the geometric relationships of both corner and surface features detected in the scan. By solving this 
    optimization problem, it updates the current pose estimation to minimize the discrepancy between the 
    current scan and its previous position.

   Parameters
   ----------
   - iterCount (int): The current iteration count in the optimization process, used to determine the adaptiveness
     and convergence criteria of the optimization.

   Returns
   -------
   bool: Indicates whether the optimization has successfully converged based on the magnitude of changes 
   in rotation and translation vectors. A return value of true suggests that the pose estimation has reached 
   a stable state with minimal adjustments required.

   - transformCur: Updates the current transformation estimates (rotation and translation) based on the 
     computed adjustments.
   - isDegenerate: Updates the flag indicating if the optimization encountered a degenerate state, affecting
     the stability and accuracy of the solution.
   - laserCloudOri, coeffSel: These are populated with selected points and corresponding coefficients for
     feature constraints used in the optimization.
*/

  int pointSelNum = laserCloudOri->points.size();

  Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
  Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
  Eigen::Matrix<float,6,6> matAtA;
  Eigen::VectorXf matB(pointSelNum);
  Eigen::Matrix<float,6,1> matAtB;
  Eigen::Matrix<float,6,1> matX;
  Eigen::Matrix<float,6,6> matP;

  float srx = sin(transformCur[0]);
  float crx = cos(transformCur[0]);
  float sry = sin(transformCur[1]);
  float cry = cos(transformCur[1]);
  float srz = sin(transformCur[2]);
  float crz = cos(transformCur[2]);
  float tx = transformCur[3];
  float ty = transformCur[4];
  float tz = transformCur[5];

  float a1 = crx * sry * srz;
  float a2 = crx * crz * sry;
  float a3 = srx * sry;
  float a4 = tx * a1 - ty * a2 - tz * a3;
  float a5 = srx * srz;
  float a6 = crz * srx;
  float a7 = ty * a6 - tz * crx - tx * a5;
  float a8 = crx * cry * srz;
  float a9 = crx * cry * crz;
  float a10 = cry * srx;
  float a11 = tz * a10 + ty * a9 - tx * a8;

  float b1 = -crz * sry - cry * srx * srz;
  float b2 = cry * crz * srx - sry * srz;
  float b3 = crx * cry;
  float b4 = tx * -b1 + ty * -b2 + tz * b3;
  float b5 = cry * crz - srx * sry * srz;
  float b6 = cry * srz + crz * srx * sry;
  float b7 = crx * sry;
  float b8 = tz * b7 - ty * b6 - tx * b5;

  float c1 = -b6;
  float c2 = b5;
  float c3 = tx * b6 - ty * b5;
  float c4 = -crx * crz;
  float c5 = crx * srz;
  float c6 = ty * c5 + tx * -c4;
  float c7 = b2;
  float c8 = -b1;
  float c9 = tx * -b2 - ty * -b1;

  for (int i = 0; i < pointSelNum; i++) {
    PointType pointOri = laserCloudOri->points[i];
    PointType coeff = coeffSel->points[i];

    float arx =
        (-a1 * pointOri.x + a2 * pointOri.y + a3 * pointOri.z + a4) * coeff.x +
        (a5 * pointOri.x - a6 * pointOri.y + crx * pointOri.z + a7) * coeff.y +
        (a8 * pointOri.x - a9 * pointOri.y - a10 * pointOri.z + a11) * coeff.z;

    float ary =
        (b1 * pointOri.x + b2 * pointOri.y - b3 * pointOri.z + b4) * coeff.x +
        (b5 * pointOri.x + b6 * pointOri.y - b7 * pointOri.z + b8) * coeff.z;

    float arz = (c1 * pointOri.x + c2 * pointOri.y + c3) * coeff.x +
                (c4 * pointOri.x - c5 * pointOri.y + c6) * coeff.y +
                (c7 * pointOri.x + c8 * pointOri.y + c9) * coeff.z;

    float atx = -b5 * coeff.x + c5 * coeff.y + b1 * coeff.z;

    float aty = -b6 * coeff.x + c4 * coeff.y + b2 * coeff.z;

    float atz = b7 * coeff.x - srx * coeff.y - b3 * coeff.z;

    float d2 = coeff.intensity;

    matA(i, 0) = arx;
    matA(i, 1) = ary;
    matA(i, 2) = arz;
    matA(i, 3) = atx;
    matA(i, 4) = aty;
    matA(i, 5) = atz;
    matB(i, 0) = -0.05 * d2;
  }

  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1, 6> matE;
    Eigen::Matrix<float,6, 6> matV;
    Eigen::Matrix<float,6, 6> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6,6> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();
    matV2 = matV;

    isDegenerate = false;
    float eignThre[6] = {10, 10, 10, 10, 10, 10};
    for (int i = 5; i >= 0; i--) {
      if (matE(0, i) < eignThre[i]) {
        for (int j = 0; j < 6; j++) {
          matV2(i, j) = 0;
        }
        isDegenerate = true;
      } else {
        break;
      }
    }
    matP = matV.inverse() * matV2;
  }

  if (isDegenerate) {
    Eigen::Matrix<float,6,1> matX2;
    matX2 = matX;
    matX = matP * matX2;
  }

  transformCur[0] += matX(0, 0);
  transformCur[1] += matX(1, 0);
  transformCur[2] += matX(2, 0);
  transformCur[3] += matX(3, 0);
  transformCur[4] += matX(4, 0);
  transformCur[5] += matX(5, 0);

  for (int i = 0; i < 6; i++) {
    if (std::isnan(transformCur[i])) transformCur[i] = 0;
  }

  float deltaR = sqrt(pow(RAD2DEG * (matX(0, 0)), 2) +
                      pow(RAD2DEG * (matX(1, 0)), 2) +
                      pow(RAD2DEG * (matX(2, 0)), 2));
  float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                      pow(matX(4, 0) * 100, 2) +
                      pow(matX(5, 0) * 100, 2));

  if (deltaR < 0.1 && deltaT < 0.1) {
    return false;
  }
  return true;
}

void FeatureAssociation::checkSystemInitialization() {

/* Summary: Prepares the system for processing by initializing last frame point clouds.
  
     Extended Description: This function is called to initialize the system's state at the start of processing. It swaps the
     current and last point clouds for both corner and surface features, ensuring that subsequent processing steps have initial
     data to work with. Additionally, it sets up KD-trees for efficient nearest neighbor searches within these point clouds. This
     initialization is crucial for the first iteration of the SLAM process, providing a baseline for feature association and
     transformation calculations. The function also publishes these initialized point clouds for visualization and further analysis.
     
     Parameters
     ----------
     laserCloudTemp : pcl::PointCloud<PointType>::Ptr
         A temporary pointer used to facilitate the swapping of point clouds.
     cornerPointsLessSharp, surfPointsLessFlat : pcl::PointCloud<PointType>::Ptr
         Point clouds containing the current frame's less sharp corner and less flat surface features, respectively.
     laserCloudCornerLast, laserCloudSurfLast : pcl::PointCloud<PointType>::Ptr
         Point clouds that will contain the last frame's corner and surface features post-initialization.
     kdtreeCornerLast, kdtreeSurfLast : pcl::KdTreeFLANN<PointType>
         KD-trees associated with the last frame's corner and surface point clouds, updated for efficient search operations.
     laserCloudCornerLastNum, laserCloudSurfLastNum : int
         Variables to store the number of points in the last corner and surface point clouds, updated post-swapping.

     Returns
     -------
     - Swaps the `cornerPointsLessSharp` with `laserCloudCornerLast` and `surfPointsLessFlat` with `laserCloudSurfLast`.
     - Updates the KD-trees `kdtreeCornerLast` and `kdtreeSurfLast` with the new last point clouds.
     - Publishes the initialized last frame point clouds to ROS topics.
     - Sets `systemInitedLM` to true, indicating that the system initialization is complete and ready for normal operation.

     The function's effects are realized through side effects on class members and external publications.

     Publishes
     ---------
     - _pub_cloud_corner_last: Publishes the last corner point cloud for visualization or further processing.
     - _pub_cloud_surf_last: Publishes the last surface point cloud for visualization or further processing.
*/

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
  kdtreeSurfLast.setInputCloud(laserCloudSurfLast);

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  sensor_msgs::msg::PointCloud2 laserCloudCornerLast2;
  pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
  laserCloudCornerLast2.header.stamp = cloudHeader.stamp;
  laserCloudCornerLast2.header.frame_id = "Lidar";
  _pub_cloud_corner_last->publish(laserCloudCornerLast2);

  sensor_msgs::msg::PointCloud2 laserCloudSurfLast2;
  pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
  laserCloudSurfLast2.header.stamp = cloudHeader.stamp;
  laserCloudSurfLast2.header.frame_id = "Lidar";
  _pub_cloud_surf_last->publish(laserCloudSurfLast2);

  systemInitedLM = true;
}

void FeatureAssociation::updateTransformation() {

  /* Summary: Iteratively refines the transformation between the current and previous scans.
  
     Extended Description: This function aims to optimize the pose estimation by aligning features from the current lidar scan
      with those from the previous scan. It conducts separate iterative processes for surface and corner features, attempting to
      minimize the transformation error. The function first tries to align surface features and, upon reaching a satisfactory
      alignment or exhausting the iteration limit, proceeds to align corner features. This iterative refinement is crucial for
      accurate localization and mapping in the SLAM process.

     Parameters
     ----------
     laserCloudCornerLastNum : int
         The count of corner features in the last scan, used to determine if there are enough features to proceed with the alignment.
     laserCloudSurfLastNum : int
         The count of surface features in the last scan, serving a similar purpose as `laserCloudCornerLastNum` for surface alignment.
     iterCount1, iterCount2 : int
         Iteration counters for the alignment processes of surface and corner features, respectively.

     Returns
     -------
     laserCloudOri : pcl::PointCloud<PointType>::Ptr
         A point cloud that gets cleared and then filled with points for which a corresponding feature has been found in the last scan.
     coeffSel : pcl::PointCloud<PointType>::Ptr
         A point cloud for storing coefficients related to the corresponding points in `laserCloudOri`, used in the transformation calculation.
     
     The function's primary goal is to update the transformation between the current and previous scans, which is achieved through 
     internal modifications rather than direct return values.

     Note
     ----
     The function returns early if the number of features in the last scan is insufficient for a reliable alignment. 
     It performs up to 25 iterations for both surface and corner feature alignments or breaks early if the calculated transformation
     converges to a stable solution.
  */

  if (laserCloudCornerLastNum < 10 || laserCloudSurfLastNum < 100) return;

  for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingSurfFeatures(iterCount1);

    if (laserCloudOri->points.size() < 10) continue;
    if (calculateTransformationSurf(iterCount1) == false) break;
  }

  for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {
    laserCloudOri->clear();
    coeffSel->clear();

    findCorrespondingCornerFeatures(iterCount2);

    if (laserCloudOri->points.size() < 10) continue;
    if (calculateTransformationCorner(iterCount2) == false) break;
  }
}

void FeatureAssociation::integrateTransformation() {

   /* Summary: Integrates the latest transformation adjustments into the global transformation state.
  
     Extended Description: This method updates the global pose (transformSum) by integrating 
     the latest incremental transformations (transformCur). It calculates the accumulated rotation and translation
     based on the current adjustments, then applies these to the global transformation. This ensures that the global pose
     is continuously updated to reflect the cumulative effect of all transformations up to the current frame.
     
     Parameters
     ----------
     rx, ry, rz : float
         Accumulated rotations around the x, y, and z axes, respectively.
     tx, ty, tz : float
         Accumulated translations along the x, y, and z axes, respectively.
     transformSum : std::array<float, 6>
         The global transformation parameters, including rotation (radians) and translation (meters).
     transformCur : std::array<float, 6>
         The current frame's incremental transformation parameters, including rotation (radians) and translation (meters).

     Returns
     -------
     - transformSum: Directly updates the global transformation parameters to include the latest adjustments.
     
     The output is the updated global transformation state stored in `transformSum`.
  */

  float rx, ry, rz, tx, ty, tz;
  AccumulateRotation(transformSum[0], transformSum[1], transformSum[2],
                     -transformCur[0], -transformCur[1], -transformCur[2], rx,
                     ry, rz);

  float x1 = cos(rz) * (transformCur[3] ) -
             sin(rz) * (transformCur[4] );
  float y1 = sin(rz) * (transformCur[3] ) +
             cos(rz) * (transformCur[4] );
  float z1 = transformCur[5];

  float x2 = x1;
  float y2 = cos(rx) * y1 - sin(rx) * z1;
  float z2 = sin(rx) * y1 + cos(rx) * z1;

  tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
  ty = transformSum[4] - y2;
  tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

  transformSum[0] = rx;
  transformSum[1] = ry;
  transformSum[2] = rz;
  transformSum[3] = tx;
  transformSum[4] = ty;
  transformSum[5] = tz;
}

void FeatureAssociation::adjustOutlierCloud() {

  /* Summary: Adjusts the orientation of points in the outlier cloud.
  
     Extended Description: This function reorients each point in the `outlierCloud` by swapping their x, y, and z coordinates. 
     This adjustment is necessary to align the outlier points with the coordinate system used by the rest of the processing pipeline, 
     ensuring consistent data interpretation across different stages of the algorithm. The adjustment primarily caters to the discrepancies 
     that might arise from sensor orientation or the initial processing steps that produce the outlier cloud.

     Parameters
     ----------
     point : PointType
         A temporary variable used to hold and adjust the coordinates of each point in the outlier cloud.
     cloudSize : int
         The total number of points in the outlier cloud, used to iterate through the cloud.

     Returns
     -------
     - outlierCloud: Each point in this cloud is modified by swapping its x, y, and z coordinates to adjust their orientation to match 
       the coordinate system used by the algorithm.

     The function's primary effect is the in-place modification of the `outlierCloud`, with each point's coordinates adjusted accordingly.
  */

  PointType point;
  int cloudSize = outlierCloud->points.size();
  for (int i = 0; i < cloudSize; ++i) {
    point.x = outlierCloud->points[i].y;
    point.y = outlierCloud->points[i].z;
    point.z = outlierCloud->points[i].x;
    point.intensity = outlierCloud->points[i].intensity;
    outlierCloud->points[i] = point;
  }
}

void FeatureAssociation::publishOdometry() {

  /* Summary: Publishes the calculated odometry and transformation to ROS topics.
  
     Extended Description: This function prepares and publishes the odometry information calculated 
     from the lidar scans, encapsulated in ROS messages. It converts the internal quaternion representation
     to ROS quaternion message format and sets the pose and orientation based on the accumulated transformations. 
     The function publishes both the odometry message and a transform message, allowing other components in the ROS ecosystem 
     to utilize the pose and orientation data for tasks like localization and mapping.

     Parameters
     ----------
     q : tf2::Quaternion
         A quaternion used for setting the initial orientation for the odometry message.
     geoQuat : geometry_msgs::msg::Quaternion
         A geometry_msgs quaternion derived from `q`, representing the orientation in the ROS message format.
     transformSum : std::array<float, 6>
         The global transformation parameters, including rotation (radians) and translation (meters), used to set the pose
         and orientation in the odometry message.

     Returns
     -------
     This function's output is the publication of odometry and transformation data to ROS topics for further utilization.

     Publishes
     ---------
     - laserOdometry: A nav_msgs::Odometry message that carries the odometry information, including pose and orientation, to a designated topic.
     - laserOdometryTrans: A geometry_msgs::TransformStamped message that carries the transformation information, 
      including translation and rotation, to the tf2 ROS system for managing transformations.
  */

  tf2::Quaternion q;
  q.setRPY(0, 0, 0); // Set the roll, pitch, yaw values as needed
  geometry_msgs::msg::Quaternion geoQuat;
  geoQuat.x = q.x();
  geoQuat.y = q.y();
  geoQuat.z = q.z();
  geoQuat.w = q.w();

  laserOdometry.header.stamp = cloudHeader.stamp;
  laserOdometry.pose.pose.orientation.x = -geoQuat.y;
  laserOdometry.pose.pose.orientation.y = -geoQuat.z;
  laserOdometry.pose.pose.orientation.z = geoQuat.x;
  laserOdometry.pose.pose.orientation.w = geoQuat.w;
  laserOdometry.pose.pose.position.x = transformSum[3];
  laserOdometry.pose.pose.position.y = transformSum[4];
  laserOdometry.pose.pose.position.z = transformSum[5];
  pubLaserOdometry->publish(laserOdometry);

  laserOdometryTrans.header.stamp = cloudHeader.stamp;
  laserOdometryTrans.transform.translation.x = transformSum[3];
  laserOdometryTrans.transform.translation.y = transformSum[4];
  laserOdometryTrans.transform.translation.z = transformSum[5];
  laserOdometryTrans.transform.rotation.x = -geoQuat.y;
  laserOdometryTrans.transform.rotation.y = -geoQuat.z;
  laserOdometryTrans.transform.rotation.z = geoQuat.x;
  laserOdometryTrans.transform.rotation.w = geoQuat.w;
  tfBroadcaster->sendTransform(laserOdometryTrans);
}

void FeatureAssociation::publishCloud() {

/* Summary: Publishes various types of processed point clouds for visualization and further processing.
  
   Extended Description: This method publishes the processed lidar point clouds, including sharp corner points,
     less sharp corner points, flat surface points, and less flat surface points. It checks for subscribers before
     publishing to avoid unnecessary processing. The point clouds are converted to ROS message format and published
     on designated topics for each point cloud type, enabling visualization in RViz or further processing by other ROS nodes.

   Parameters
   ----------
     laserCloudOutMsg : sensor_msgs::msg::PointCloud2
         The ROS message used for publishing the point clouds.
     pub : rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
         The ROS publisher objects for each type of point cloud.

    Returns
    -------
     - Publishes point clouds to their respective ROS topics based on the subscription count to optimize processing.
       
     The function's output is the publication of various processed point clouds to ROS topics for external use.

   Publishes
   ---------
   - pubCornerPointsSharp for sharp corner points.
   - pubCornerPointsLessSharp for less sharp corner points.
   - pubSurfPointsFlat for flat surface points.
   - pubSurfPointsLessFlat for less flat surface points.
*/

  sensor_msgs::msg::PointCloud2 laserCloudOutMsg;

  auto Publish = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                     const pcl::PointCloud<PointType>::Ptr &cloud) {
    if (pub->get_subscription_count() != 0) {
      pcl::toROSMsg(*cloud, laserCloudOutMsg);
      laserCloudOutMsg.header.stamp = cloudHeader.stamp;
      laserCloudOutMsg.header.frame_id = "Lidar";
      pub->publish(laserCloudOutMsg);
    }
  };

  Publish(pubCornerPointsSharp, cornerPointsSharp);
  Publish(pubCornerPointsLessSharp, cornerPointsLessSharp);
  Publish(pubSurfPointsFlat, surfPointsFlat);
  Publish(pubSurfPointsLessFlat, surfPointsLessFlat);
}

void FeatureAssociation::publishCloudsLast() {

/* Summary: Publishes the last frame's processed point clouds for mapping optimization.

   Extended Description: This function readies the last frame's sharp and less sharp corner points,
     as well as flat and less flat surface points, for mapping optimization by applying a transformation
     to align them with the end of the scan frame. The transformed clouds are then swapped with the current
     last frame clouds and published if there are subscribers. This method ensures the mapping optimization
     module receives the most recent data, aiding accurate and up-to-date map construction.

   Parameters
   ----------
     cornerPointsLessSharpNum : int
         Number of less sharp corner points, used to iterate and apply transformations.
     surfPointsLessFlatNum : int
         Number of less flat surface points, used similarly for transformation.
     laserCloudTemp : pcl::PointCloud<PointType>::Ptr
         Temporary pointer for swapping point clouds.
     frameCount : int
         Counter to manage publishing frequency based on `skipFrameNum`.

   Returns
   -------
     - laserCloudCornerLast, laserCloudSurfLast: Updates these clouds with transformed points for the last frame.
     - outlierCloud: Adjusts the outlier cloud points' orientation for consistency.

     The function's output is the publication of various processed point clouds to ROS topics for external use.

   Publishes
   ---------
     - _pub_outlier_cloudLast: Publishes adjusted outlier points.
     - _pub_cloud_corner_last: Publishes the last frame's corner points.
     - _pub_cloud_surf_last: Publishes the last frame's surface points.

   Note
   ----
   - Uses `frameCount` to manage the frequency of publishing based on `skipFrameNum`.
   - Utilizes internal class members such as `laserCloudCornerLast`, `laserCloudSurfLast`, and `outlierCloud`
     to store and publish point cloud data.
*/


  int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
  for (int i = 0; i < cornerPointsLessSharpNum; i++) {
    TransformToEnd(&cornerPointsLessSharp->points[i],
                   &cornerPointsLessSharp->points[i]);
  }

  int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
  for (int i = 0; i < surfPointsLessFlatNum; i++) {
    TransformToEnd(&surfPointsLessFlat->points[i],
                   &surfPointsLessFlat->points[i]);
  }

  pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerLastNum = laserCloudCornerLast->points.size();
  laserCloudSurfLastNum = laserCloudSurfLast->points.size();

  if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
    kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast.setInputCloud(laserCloudSurfLast);
  }

  frameCount++;
  adjustOutlierCloud();

  if (frameCount >= skipFrameNum + 1) {
    frameCount = 0;
    sensor_msgs::msg::PointCloud2 cloudTemp;

    auto Publish = [&](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                       const pcl::PointCloud<PointType>::Ptr &cloud) {
      if (pub->get_subscription_count() != 0) {
        pcl::toROSMsg(*cloud, cloudTemp);
        cloudTemp.header.stamp = cloudHeader.stamp;
        cloudTemp.header.frame_id = "Lidar";
        pub->publish(cloudTemp);
      }
    };

    Publish(_pub_outlier_cloudLast, outlierCloud);
    Publish(_pub_cloud_corner_last, laserCloudCornerLast);
    Publish(_pub_cloud_surf_last, laserCloudSurfLast);
  }
}

void FeatureAssociation::runFeatureAssociation() {

/* Summary: Orchestrates the feature association process for LiDAR SLAM.

   Extended Description: This function acts as the core of the feature association process, 
   iterating over incoming LiDAR scans and performing a series of steps to extract features, 
   associate them, and publish results for mapping optimization. It starts by adjusting for 
   distortion in the scan, calculates the smoothness of points, marks occluded points, and extracts 
   features. If the system is not initialized, it checks for initialization conditions. Otherwise, 
   it updates and integrates transformations based on extracted features, publishes odometry data, 
   and prepares the last frame's processed point clouds for mapping optimization. This cycle 
   repeats for each scan, contributing to the SLAM process by providing essential data for constructing
   and optimizing a map of the environment.

   Parameters
   ----------
   - Engages with multiple internal data structures to store and process LiDAR data.
   - Utilizes `_cycle_count` to manage the frequency of output for mapping optimization, controlled by `_mapping_frequency_div`.

   Returns
   -------
   - Processes and transforms incoming LiDAR data to extract features and associate them with the environment.
   - Updates internal state based on the current scan and the transformations calculated.
   - Publishes odometry and processed point clouds for visualization and mapping optimization.

   Note
   ----
   This function is designed to run in a loop, continuously processing incoming LiDAR scans until the application is terminated.
*/

  while (rclcpp::ok()) {
    ProjectionOut projection;
    _input_channel.receive(projection);

    if( !rclcpp::ok() ) break;

    //--------------
    outlierCloud = projection.outlier_cloud;
    segmentedCloud = projection.segmented_cloud;
    segInfo = std::move(projection.seg_msg);

    cloudHeader = segInfo.header;

    /**  1. Feature Extraction  */
    adjustDistortion();

    calculateSmoothness();

    markOccludedPoints();

    extractFeatures();

    publishCloud();  // cloud for visualization

    // Feature Association
    if (!systemInitedLM) {
      checkSystemInitialization();
      continue;
    }

    updateTransformation();

    integrateTransformation();

    publishOdometry();

    publishCloudsLast();  // cloud to mapOptimization

    //--------------
    _cycle_count++;

    if (static_cast<int>(_cycle_count) == _mapping_frequency_div) {
      _cycle_count = 0;
      AssociationOut out;
      out.cloud_corner_last.reset(new pcl::PointCloud<PointType>());
      out.cloud_surf_last.reset(new pcl::PointCloud<PointType>());
      out.cloud_outlier_last.reset(new pcl::PointCloud<PointType>());

      *out.cloud_corner_last = *laserCloudCornerLast;
      *out.cloud_surf_last = *laserCloudSurfLast;
      *out.cloud_outlier_last = *outlierCloud;

      out.laser_odometry = laserOdometry;

      _output_channel.send(std::move(out));
    }
  }
}