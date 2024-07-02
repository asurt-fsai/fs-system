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
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar
//   Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems
//      (IROS). October 2018.

#include "mapOptimization.h"
#include <future>
#include "StatusPublisher.h"

using namespace gtsam;

const std::string PARAM_ENABLE_LOOP = "mapping.enable_loop_closure";
const std::string PARAM_SEARCH_RADIUS = "mapping.surrounding_keyframe_search_radius";
const std::string PARAM_SEARCH_NUM = "mapping.surrounding_keyframe_search_num";
const std::string PARAM_HISTORY_SEARCH_RADIUS = "mapping.history_keyframe_search_radius";
const std::string PARAM_HISTORY_SEARCH_NUM = "mapping.history_keyframe_search_num";
const std::string PARAM_HISTORY_SCORE = "mapping.history_keyframe_fitness_score";
const std::string PARAM_GLOBAL_SEARCH_RADIUS = "mapping.global_map_visualization_search_radius";

MapOptimization::MapOptimization(const std::string &name, Channel<AssociationOut> &input_channel)
    : Node(name), _input_channel(input_channel), _publish_global_signal(false), _loop_closure_signal(false), 
    statusPublisher("/status/mapOptimization", this) {

  /* Initializes the MapOptimization node for LiDAR odometry and mapping optimization. 
     This constructor sets up the optimization framework, publishers for various point clouds and odometry data, 
     and configures downsampling filters for efficient processing.

   Attributes
   ----------
   _input_channel : Channel<AssociationOut>
       Channel for receiving associated data including corner points, surface points, and outlier points from the LiDAR odometry.
   _publish_global_signal : Signal<bool>
       Controls the periodic publishing of the global map.
   _loop_closure_signal : Signal<bool>
       Triggers loop closure detection and processing.
   pubKeyPoses : Publisher
       For publishing the key poses of the map.
   pubLaserCloudSurround : Publisher
       For publishing the surrounding laser cloud points.
   pubOdomAftMapped : Publisher
       For publishing odometry information after mapping adjustments.
   pubOdomAftMappedAdjusted: Publisher
       For publishing odometry information after adjusting the orientation
   downSizeFilterCorner : pcl::VoxelGrid<PointType>
       Downsampling filter for corner points.
   downSizeFilterSurf : pcl::VoxelGrid<PointType>
       Downsampling filter for surface points.
   parameters : ISAM2Params
       Configuration parameters for the ISAM2 optimization algorithm.

   Methods
   -------
   MapOptimization(const std::string &name, Channel<AssociationOut>& input_channel):
       Constructs a MapOptimization object and initializes its parameters, publishers, and subscribers. 
       It sets up the ISAM2 optimizer with predefined parameters and initializes downsampling filters for point clouds.
       Parameters include the node name and an input channel for receiving associated data from the LiDAR odometry component.

   Note
   ----
   This class is responsible for performing map optimization in LiDAR odometry and mapping (LOAM) systems. 
   It processes incoming LiDAR data to optimize the map and detect loop closures, significantly improving the accuracy and 
   consistency of the map over time.
*/

  // Publish starting status
  statusPublisher.starting();

  ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  isam = new ISAM2(parameters);

  // Declare Topic Parameters
  this->declare_parameter("topics.keyPoseOrigin", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topics.laserCloudSurround", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topics.aftMappedToInit", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topics.historyCloud", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topics.correctedCloud", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topics.recentCloud", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topics.aftMappedAdjusted", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topics.loopClosureFlag", rclcpp::PARAMETER_STRING);

  // Declare Frame Parameters
  this->declare_parameter("frames.cameraInit", rclcpp::PARAMETER_STRING);
  this->declare_parameter("frames.aftMapped", rclcpp::PARAMETER_STRING);
  this->declare_parameter("frames.aftAdjusted", rclcpp::PARAMETER_STRING);
  this->declare_parameter("frames.mapAdjusted", rclcpp::PARAMETER_STRING);

  // Get Topic Parameters
  this->get_parameter("topics.keyPoseOrigin", _keyPoseOrigin);
  this->get_parameter("topics.laserCloudSurround", _laserCloudSurround);
  this->get_parameter("topics.aftMappedToInit", _aftMappedToInit);
  this->get_parameter("topics.historyCloud", _historyCloud);
  this->get_parameter("topics.correctedCloud", _correctedCloud);
  this->get_parameter("topics.recentCloud", _recentCloud);
  this->get_parameter("topics.aftMappedAdjusted", _aftMappedAdjusted);
  this->get_parameter("topics.loopClosureFlag", _loopClosureFlag);

  // Get Frame Parameters
  this->get_parameter("frames.cameraInit", _cameraInit);
  this->get_parameter("frames.aftMapped", _aftMapped);
  this->get_parameter("frames.aftAdjusted", _aftAdjusted);
  this->get_parameter("frames.mapAdjusted", _mapAdjusted);

  pubKeyPoses = this->create_publisher<sensor_msgs::msg::PointCloud2>(_keyPoseOrigin, 2);
  pubLaserCloudSurround = this->create_publisher<sensor_msgs::msg::PointCloud2>(_laserCloudSurround, 2);
  pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>(_aftMappedToInit, 5);
  pubHistoryKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>(_historyCloud, 2);
  pubIcpKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>(_correctedCloud, 2);
  pubRecentKeyFrames = this->create_publisher<sensor_msgs::msg::PointCloud2>(_recentCloud, 2);
  pubOdomAftMappedAdjusted = this->create_publisher<nav_msgs::msg::Odometry>(_aftMappedAdjusted, 5);
  pubLoopClosureStatus = this->create_publisher<std_msgs::msg::Bool>(_loopClosureFlag, 2);

  tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
  downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
  downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

  // for histor key frames of loop closure
  downSizeFilterHistoryKeyFrames.setLeafSize(0.4, 0.4, 0.4);
  // for surrounding key poses of scan-to-map optimization
  downSizeFilterSurroundingKeyPoses.setLeafSize(1.0, 1.0, 1.0);

  // for global map visualization
  downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0);
  // for global map visualization
  downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4);

  odomAftMapped.header.frame_id = _cameraInit;
  odomAftMapped.child_frame_id = _aftMapped;

  aftMappedTrans.header.frame_id = _cameraInit;
  aftMappedTrans.child_frame_id = _aftMapped;

  odomAftMappedAdjusted.header.frame_id = _mapAdjusted;
  odomAftMappedAdjusted.child_frame_id = _aftAdjusted;

  aftMappedTrans.header.frame_id = _mapAdjusted;
  aftMappedTrans.child_frame_id = _aftAdjusted;

  // Declare parameters
  this->declare_parameter(PARAM_ENABLE_LOOP, true);
  this->declare_parameter(PARAM_SEARCH_RADIUS, rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(PARAM_SEARCH_NUM, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_HISTORY_SEARCH_RADIUS, rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(PARAM_HISTORY_SEARCH_NUM, rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(PARAM_HISTORY_SCORE, rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(PARAM_GLOBAL_SEARCH_RADIUS, rclcpp::PARAMETER_DOUBLE);

  // Read parameters
  if (!this->get_parameter(PARAM_ENABLE_LOOP, _loop_closure_enabled)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ENABLE_LOOP.c_str());
  }
  if (!this->get_parameter(PARAM_SEARCH_RADIUS, _surrounding_keyframe_search_radius)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEARCH_RADIUS.c_str());
  }
  if (!this->get_parameter(PARAM_SEARCH_NUM, _surrounding_keyframe_search_num)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEARCH_NUM.c_str());
  }
  if (!this->get_parameter(PARAM_HISTORY_SEARCH_RADIUS, _history_keyframe_search_radius)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HISTORY_SEARCH_RADIUS.c_str());
  }
  if (!this->get_parameter(PARAM_HISTORY_SEARCH_NUM, _history_keyframe_search_num)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HISTORY_SEARCH_NUM.c_str());
  }
  if (!this->get_parameter(PARAM_HISTORY_SCORE, _history_keyframe_fitness_score)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HISTORY_SCORE.c_str());
  }
  if (!this->get_parameter(PARAM_GLOBAL_SEARCH_RADIUS, _global_map_visualization_search_radius)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_GLOBAL_SEARCH_RADIUS.c_str());
  }

  allocateMemory();

  _publish_global_thread = std::thread(&MapOptimization::publishGlobalMapThread, this);
  _loop_closure_thread = std::thread(&MapOptimization::loopClosureThread, this);
  _run_thread = std::thread(&MapOptimization::run, this);

}

MapOptimization::~MapOptimization()
{
  /* Destructor for the MapOptimization class.

   Safely terminates background threads and signals used for global map publishing and loop closure processing. 
   It ensures all resources are properly released and that all operations are completed before the object is destroyed.

   Attributes
   ----------
   None.

   Methods
   -------
   MapOptimization::~MapOptimization():
       Signals the termination of processing loops for map optimization, loop closure detection, and global map publishing. 
       Waits for all background threads to complete their execution to ensure a clean shutdown and resource deallocation.

   Note
   ----
   This method ensures that the MapOptimization object is destructed without leaving any background tasks running, 
   adhering to proper resource management practices in a multi-threaded environment.
*/

  _input_channel.send({});
  _run_thread.join();

  _publish_global_signal.send(false);
  _publish_global_thread.join();

  _loop_closure_signal.send(false);
  _loop_closure_thread.join();
}

void MapOptimization::allocateMemory() {

  /* Summary: Prepares all necessary point clouds and matrices for the map optimization process.

   Extended Description: Allocates and initializes memory for point clouds used to store key poses, 
   laser measurements, and intermediate results of the mapping process. It sets up data structures for both raw and downsampled point clouds, 
   ensuring efficient processing throughout the map optimization and loop closure detection. Additionally, it initializes 
   transformation matrices and variables that are crucial for pose estimation and map updates.

   Parameters
   ----------
   None.

   Returns
   -------
   This function solely prepares the internal state of the MapOptimization object, making it ready for processing incoming LiDAR data
    and performing SLAM operations.
*/

  cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
  cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

  surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
  surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

  laserCloudCornerLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudSurfLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudCornerLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner featuer set
                                          // from odoOptimization
  laserCloudSurfLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization
  laserCloudOutlierLast.reset(
      new pcl::PointCloud<PointType>());  // corner feature set from
                                          // odoOptimization
  laserCloudOutlierLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled corner feature set
                                          // from odoOptimization
  laserCloudSurfTotalLast.reset(
      new pcl::PointCloud<PointType>());  // surf feature set from
                                          // odoOptimization
  laserCloudSurfTotalLastDS.reset(
      new pcl::PointCloud<PointType>());  // downsampled surf featuer set from
                                          // odoOptimization

  laserCloudOri.reset(new pcl::PointCloud<PointType>());
  coeffSel.reset(new pcl::PointCloud<PointType>());

  laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
  laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
  laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

  nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
  nearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  nearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

  latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  latestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
  latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

  globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
  globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
  globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
  globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

  timeLaserOdometry = this->now();

  for (int i = 0; i < 6; ++i) {
    transformLast[i] = 0;
    transformSum[i] = 0;
    transformIncre[i] = 0;
    transformTobeMapped[i] = 0;
    transformBefMapped[i] = 0;
    transformAftMapped[i] = 0;
  }


  matA0.setZero();
  matB0.fill(-1);
  matX0.setZero();

  matA1.setZero();
  matD1.setZero();
  matV1.setZero();

  isDegenerate = false;
  matP.setZero();

  laserCloudCornerFromMapDSNum = 0;
  laserCloudSurfFromMapDSNum = 0;
  laserCloudCornerLastDSNum = 0;
  laserCloudSurfLastDSNum = 0;
  laserCloudOutlierLastDSNum = 0;
  laserCloudSurfTotalLastDSNum = 0;

  potentialLoopFlag = false;
  aLoopIsClosed = false;

  latestFrameID = 0;
}


void MapOptimization::publishGlobalMapThread()
{
  /* Summary: Continuously checks and publishes the global map when signaled.

    Extended Description: Operates within a dedicated thread to periodically check for a signal indicating whether
    the global map is ready for publishing. Once the signal is received, it triggers the publishing of the global map 
    to ensure the most current map data is available for use or visualization. This method allows for the efficient updating
    and distribution of the global map without blocking the main processing threads of map optimization and loop closure.

    Parameters
    ----------
    - ready : bool
        A flag received from `_publish_global_signal` indicating whether the global map is ready to be published.

    Returns
    -------
    The function's primary role is to await a signal and then execute the map publishing process accordingly. 
    By decoupling the map publishing process from the main computational thread, this method enhances the system's ability to 
    maintain real-time performance while ensuring the global map's availability is kept up-to-date.
  */

  while(rclcpp::ok())
  {
    bool ready;
    _publish_global_signal.receive(ready);
    if(ready){
      publishGlobalMap();
    }
  }
}

void MapOptimization::loopClosureThread()
{
  /* Summary: Monitors and executes loop closure operations when signaled.

    Extended Description: This method runs in a dedicated background thread, continuously checking for signals 
    to initiate the loop closure process. Upon receiving a positive signal and with loop closure enabled, 
    it invokes the `performLoopClosure` method. This approach ensures that loop closure detection and processing 
    are managed efficiently without interrupting the core mapping operations.

    Parameters
    ----------
    - ready : bool
        A boolean value received from `_loop_closure_signal`, indicating whether a loop closure operation should be attempted. 

    Returns
    -------
    The function serves to asynchronously trigger loop closure processes based on external signaling and internal configuration.
    This method is key to the modular and responsive design of the mapping system, allowing for loop closures to be detected and 
    processed in parallel with ongoing map optimization tasks.
  */

  while(rclcpp::ok())
  {
    bool ready;
    _loop_closure_signal.receive(ready);
    if(ready && _loop_closure_enabled){
      performLoopClosure();
    }
  }
}

void MapOptimization::transformAssociateToMap() {

  /* Summary: Calculates the transformation needed to align the current scan with the global map frame.

   Extended Description: This function updates the `transformTobeMapped` variable to ensure the current scan's pose
   is accurately aligned with the global map. It uses a combination of the current and 
   previous transformations (`transformSum`, `transformBefMapped`, and `transformAftMapped`) to compute the necessary adjustments. 
   The method applies trigonometric operations to reconcile local transformations with the global coordinate system, 
   facilitating consistent map integration and pose estimation.

   Parameters
   ----------
   - transformSum : array
       Cumulative transformation from the initial pose to the current pose.
   - transformBefMapped : array
       The pose before the last map optimization.
   - transformAftMapped : array
       The pose after the last map optimization.
   These parameters are modified within the function to calculate:
   - x1, y1, z1, x2, y2, z2 : float
       Intermediate variables for transformation adjustments.
   - sbcx, cbcx, sbcy, cbcy, sbcz, cbcz : float
       Sine and cosine values of the cumulative rotation angles.
   - sblx, cblx, sbly, cbly, sblz, cblz : float
       Sine and cosine values of the rotation angles before mapping.
   - salx, calx, saly, caly, salz, calz : float
       Sine and cosine values of the rotation angles after mapping.

   Returns
   -------
   The function directly modifies `transformTobeMapped`, which is an internal variable representing the adjusted 
   current pose in the global map's frame of reference.
   The precise adjustment of the robot's pose within the global map is crucial for the accuracy of subsequent 
   mapping and localization processes. This function ensures that each local scan is correctly positioned within the larger map context, 
   supporting the overall coherence and reliability of the map.
*/

  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) -
             sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) +
             cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz +
                       calx * calz * cblx * cblz) -
              cbcx * sbcy *
                  (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
                   calx * salz * (cbly * cblz + sblx * sbly * sblz) +
                   cblx * salx * sbly) -
              cbcx * cbcy *
                  (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
                   calx * calz * (sbly * sblz + cbly * cblz * sblx) +
                   cblx * cbly * salx);
  transformTobeMapped[0] = -asin(srx);

  float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) -
                         cblx * sblz * (caly * calz + salx * saly * salz) +
                         calx * saly * sblx) -
                 cbcx * cbcy *
                     ((caly * calz + salx * saly * salz) *
                          (cblz * sbly - cbly * sblx * sblz) +
                      (caly * salz - calz * salx * saly) *
                          (sbly * sblz + cbly * cblz * sblx) -
                      calx * cblx * cbly * saly) +
                 cbcx * sbcy *
                     ((caly * calz + salx * saly * salz) *
                          (cbly * cblz + sblx * sbly * sblz) +
                      (caly * salz - calz * salx * saly) *
                          (cbly * sblz - cblz * sblx * sbly) +
                      calx * cblx * saly * sbly);
  float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) -
                         cblx * cblz * (saly * salz + caly * calz * salx) +
                         calx * caly * sblx) +
                 cbcx * cbcy *
                     ((saly * salz + caly * calz * salx) *
                          (sbly * sblz + cbly * cblz * sblx) +
                      (calz * saly - caly * salx * salz) *
                          (cblz * sbly - cbly * sblx * sblz) +
                      calx * caly * cblx * cbly) -
                 cbcx * sbcy *
                     ((saly * salz + caly * calz * salx) *
                          (cbly * sblz - cblz * sblx * sbly) +
                      (calz * saly - caly * salx * salz) *
                          (cbly * cblz + sblx * sbly * sblz) -
                      calx * caly * cblx * sbly);
  transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                 crycrx / cos(transformTobeMapped[0]));

  float srzcrx =
      (cbcz * sbcy - cbcy * sbcx * sbcz) *
          (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
           calx * calz * (sbly * sblz + cbly * cblz * sblx) +
           cblx * cbly * salx) -
      (cbcy * cbcz + sbcx * sbcy * sbcz) *
          (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
           calx * salz * (cbly * cblz + sblx * sbly * sblz) +
           cblx * salx * sbly) +
      cbcx * sbcz *
          (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  float crzcrx =
      (cbcy * sbcz - cbcz * sbcx * sbcy) *
          (calx * calz * (cbly * sblz - cblz * sblx * sbly) -
           calx * salz * (cbly * cblz + sblx * sbly * sblz) +
           cblx * salx * sbly) -
      (sbcy * sbcz + cbcy * cbcz * sbcx) *
          (calx * salz * (cblz * sbly - cbly * sblx * sblz) -
           calx * calz * (sbly * sblz + cbly * cblz * sblx) +
           cblx * cbly * salx) +
      cbcx * cbcz *
          (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
  transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                 crzcrx / cos(transformTobeMapped[0]));

  x1 = cos(transformTobeMapped[2]) * transformIncre[3] -
       sin(transformTobeMapped[2]) * transformIncre[4];
  y1 = sin(transformTobeMapped[2]) * transformIncre[3] +
       cos(transformTobeMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  transformTobeMapped[3] =
      transformAftMapped[3] -
      (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
  transformTobeMapped[4] = transformAftMapped[4] - y2;
  transformTobeMapped[5] =
      transformAftMapped[5] -
      (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void MapOptimization::transformUpdate() {

  /* Summary: Updates transformation variables after each map optimization cycle.

   Extended Description: This function synchronizes the transformation variables post-optimization. 
   It assigns the cumulative transformation (`transformSum`) to the pre-optimization 
   transformation variable (`transformBefMapped`) and the current optimized transformation (`transformTobeMapped`) to the post-optimization
   transformation variable (`transformAftMapped`). This update is crucial for maintaining an accurate and continuous record of 
   the vehicle's pose relative to the global map across successive optimization cycles.

   Parameters
   ----------
   - transformSum : array
       The cumulative transformation from the initial pose to the current pose before the latest optimization.
   - transformTobeMapped : array
       The current pose's transformation as calculated in the latest optimization cycle.

   Parameters Updated
   --------------------------
   - transformBefMapped : array
       Updated to reflect the vehicle's pose before the latest optimization, for use in the next optimization cycle.
   - transformAftMapped : array
       Updated to reflect the vehicle's current pose after the latest optimization, serving as a reference for subsequent operations.

   Returns
   -------
   This function modifies the internal state of the `transformBefMapped` and `transformAftMapped` variables to reflect
   the most recent pose adjustments. Ensuring the accuracy of these transformation variables is essential for the consistency
   of map optimization and effective loop closure detection, contributing to the overall SLAM process's reliability.
*/

  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = transformSum[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
}

void MapOptimization::updatePointAssociateToMapSinCos() {

/* Summary: Updates sine and cosine values for roll, pitch, and yaw based on the current transformation.

   Extended Description: This function calculates the sine and cosine values for the roll, pitch, and yaw components of the `transformTobeMapped` 
   transformation. These precomputed values are used to optimize the point association process to the map by reducing repetitive calculations. 
   Additionally, it updates the translation components (tX, tY, tZ) to reflect the current position in the map frame.

   Parameters
   ----------
   - transformTobeMapped[0] : float
       Roll angle of the current transformation.
   - transformTobeMapped[1] : float
       Pitch angle of the current transformation.
   - transformTobeMapped[2] : float
       Yaw angle of the current transformation.
   - transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5] : float
       Translation components of the current transformation in the map frame.
   - cRoll, sRoll : float
       Cosine and sine of the roll angle.
   - cPitch, sPitch : float
       Cosine and sine of the pitch angle.
   - cYaw, sYaw : float
       Cosine and sine of the yaw angle.
   - tX, tY, tZ : float
       Updated translation components in the map frame.

   Returns
   -------
   The function updates internal variables for sine, cosine, and translation, facilitating efficient point association to the map.
   By precalculating these trigonometric and translational components, the function streamlines subsequent transformations of point cloud data, 
   contributing to the efficiency of the map optimization process.
*/
  cRoll = cos(transformTobeMapped[0]);
  sRoll = sin(transformTobeMapped[0]);

  cPitch = cos(transformTobeMapped[1]);
  sPitch = sin(transformTobeMapped[1]);

  cYaw = cos(transformTobeMapped[2]);
  sYaw = sin(transformTobeMapped[2]);

  tX = transformTobeMapped[3];
  tY = transformTobeMapped[4];
  tZ = transformTobeMapped[5];
}

void MapOptimization::pointAssociateToMap(PointType const *const pi,
                                          PointType *const po) {
                            
  /* Summary: Transforms a point from the sensor frame to the map frame.

   Extended Description: This function applies the current estimated transformation (including rotation and translation) 
   to a point in the sensor frame, effectively placing it in the map's frame of reference. It utilizes precomputed sine and cosine values 
   for the roll, pitch, and yaw of the transformation to efficiently rotate the point before applying the translation.

   Parameters
   ----------
   pi : PointType const *const
       The input point in the sensor frame that needs to be transformed.
   po : PointType *const
       The output point in the map frame after transformation.

   - cYaw, sYaw : float
       Cosine and sine values for the yaw component of the transformation.
   - cRoll, sRoll : float
       Cosine and sine values for the roll component of the transformation.
   - cPitch, sPitch : float
       Cosine and sine values for the pitch component of the transformation.
   - tX, tY, tZ : float
       Translation components of the transformation.

   Returns
   -------
   The function directly modifies the `po` point to reflect its new position and orientation in the map frame.
   This transformation is essential for integrating individual sensor measurements into a cohesive global map, 
   allowing for accurate localization and mapping by aligning sensor data with the global coordinate system.
*/
  float x1 = cYaw * pi->x - sYaw * pi->y;
  float y1 = sYaw * pi->x + cYaw * pi->y;
  float z1 = pi->z;

  float x2 = x1;
  float y2 = cRoll * y1 - sRoll * z1;
  float z2 = sRoll * y1 + cRoll * z1;

  po->x = cPitch * x2 + sPitch * z2 + tX;
  po->y = y2 + tY;
  po->z = -sPitch * x2 + cPitch * z2 + tZ;
  po->intensity = pi->intensity;
}

void MapOptimization::updateTransformPointCloudSinCos(PointTypePose *tIn) {

  /* Summary: Updates the sine and cosine values for a given pose transformation.

   Extended Description: For a given input pose, this function calculates and updates the cosine and sine values of 
   the roll, pitch, and yaw angles. Additionally, it updates the translation components (tInX, tInY, tInZ) based on the input pose. 
   These calculations are crucial for efficiently transforming points between the sensor frame and the map frame, especially in operations 
   involving large numbers of points where computational efficiency is paramount.

   Parameters
   ----------
   tIn : PointTypePose * 
       The input pose containing roll, pitch, yaw, and translation components from which sine and cosine values are derived.

   - ctRoll, stRoll : float
       Updated cosine and sine of the roll angle derived from `tIn`.
   - ctPitch, stPitch : float
       Updated cosine and sine of the pitch angle derived from `tIn`.
   - ctYaw, stYaw : float
       Updated cosine and sine of the yaw angle derived from `tIn`.
   - tInX, tInY, tInZ : float
       Updated translation components derived from `tIn`.

   Returns
   -------
   The function's primary purpose is to update internal variables for sine, cosine, and translation based on the input pose, 
   facilitating subsequent point transformations. By precalculating these trigonometric and translational components, 
   the function optimizes the transformation process of point clouds, enhancing the overall efficiency of mapping operations.
*/

  ctRoll = cos(tIn->roll);
  stRoll = sin(tIn->roll);

  ctPitch = cos(tIn->pitch);
  stPitch = sin(tIn->pitch);

  ctYaw = cos(tIn->yaw);
  stYaw = sin(tIn->yaw);

  tInX = tIn->x;
  tInY = tIn->y;
  tInZ = tIn->z;
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn) {

  /* Summary: Transforms a point cloud from the sensor frame to the map frame using custom trigonometric transformations.

   Extended Description: This function manually transforms each point in the input cloud from the sensor frame to the map frame
   based on precomputed sine and cosine values for roll, pitch, and yaw, along with translation offsets. The manual transformation 
   is preferred over PCL's built-in methods due to accuracy concerns. It involves rotating the points according to the 
   yaw, pitch, and roll angles before applying the translation.

   Parameters
   ----------
   cloudIn : pcl::PointCloud<PointType>::Ptr
       The input point cloud in the sensor frame that needs to be transformed to the map frame.

   - ctYaw, stYaw : float
       Cosine and sine of the yaw angle.
   - ctRoll, stRoll : float
       Cosine and sine of the roll angle.
   - ctPitch, stPitch : float
       Cosine and sine of the pitch angle.
   - tInX, tInY, tInZ : float
       Translation components of the transformation.

   Returns
   -------
   pcl::PointCloud<PointType>::Ptr
       The transformed point cloud in the map frame.

   The function iterates over each point in the input cloud, applying the rotation and translation to move it into the map frame. 
   This process is critical for integrating sensor data into a coherent map representation, facilitating accurate localization and mapping.
*/

  // !!! DO NOT use pcl for point cloud transformation, results are not
  // accurate Reason: unknown
  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType *pointFrom;
  PointType pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
    float y1 = stYaw * pointFrom->x + ctYaw * pointFrom->y;
    float z1 = pointFrom->z;

    float x2 = x1;
    float y2 = ctRoll * y1 - stRoll * z1;
    float z2 = stRoll * y1 + ctRoll * z1;

    pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
    pointTo.y = y2 + tInY;
    pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
    pointTo.intensity = pointFrom->intensity;

    cloudOut->points[i] = pointTo;
  }
  return cloudOut;
}

pcl::PointCloud<PointType>::Ptr MapOptimization::transformPointCloud(
    pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn) {

  /* Summary: Transforms a given point cloud to the map frame using a specific pose.

   Extended Description: This function takes an input point cloud and a transformation pose, then applies a manual transformation process
   to each point. It rotates the points based on the provided pose's yaw, pitch, and roll, followed by translation to the new position. 
   This method allows for flexible point cloud transformations using arbitrary poses, facilitating various SLAM operations such as integrating
   scans into submaps or global maps.

   Parameters
   ----------
   cloudIn : pcl::PointCloud<PointType>::Ptr
       The input point cloud that will be transformed.
   transformIn : PointTypePose*
       The pose containing the rotation (roll, pitch, yaw) and translation (x, y, z) used for transforming the cloud.

   - x1, y1, z1 : float
       Intermediate variables representing the coordinates of a point after applying the rotation around the yaw axis.
   - x2, y2, z2 : float
       Intermediate variables representing the coordinates of a point after applying the rotation around the roll axis, 
       following the initial yaw rotation.
   - pointFrom : PointType*
       Pointer to the current point in the input cloud being transformed.
   - pointTo : PointType
       Variable holding the transformed coordinates of pointFrom, to be placed into the output cloud.

   Returns
   -------
   pcl::PointCloud<PointType>::Ptr
       A pointer to the newly created point cloud that has been transformed to the map frame according to transformIn.

   This function is crucial for operations that require repositioning point clouds based on new or updated poses, 
   such as after pose graph optimization or when merging submaps into a larger map context.
*/

  pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

  PointType *pointFrom;
  PointType pointTo;

  int cloudSize = cloudIn->points.size();
  cloudOut->resize(cloudSize);

  for (int i = 0; i < cloudSize; ++i) {
    pointFrom = &cloudIn->points[i];
    float x1 = cos(transformIn->yaw) * pointFrom->x -
               sin(transformIn->yaw) * pointFrom->y;
    float y1 = sin(transformIn->yaw) * pointFrom->x +
               cos(transformIn->yaw) * pointFrom->y;
    float z1 = pointFrom->z;

    float x2 = x1;
    float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
    float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

    pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 +
                transformIn->x;
    pointTo.y = y2 + transformIn->y;
    pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 +
                transformIn->z;
    pointTo.intensity = pointFrom->intensity;

    cloudOut->points[i] = pointTo;
  }
  return cloudOut;
}


void MapOptimization::publishTF() {

  /* Summary: Publishes the transformed pose as a TF (Transform Frame) and an Odometry message.

   Extended Description: This function broadcasts the latest optimized pose (`transformAftMapped`) as both a transformation frame (TF)
   and an odometry message. It uses the quaternion representation for orientation to comply with ROS standards. 
   This process involves converting the roll, pitch, and yaw angles into a quaternion and then setting the odometry message's orientation
   and position fields accordingly. The TF is broadcasted to update the robot's pose in the global reference frame, 
   while the odometry message provides detailed pose and velocity information.

   Parameters
   ----------
   - transformAftMapped : array
       Contains the current pose in terms of x, y, z, roll, pitch, and yaw after map optimization.
   - timeLaserOdometry : Time
       The timestamp associated with the current laser odometry reading.
   - q : tf2::Quaternion
       Quaternion representing the orientation calculated from `transformAftMapped`.
   - geoQuat : geometry_msgs::msg::Quaternion
       ROS message compatible quaternion for orientation.
   - odomAftMapped : nav_msgs::msg::Odometry
       The odometry message being published, containing the current pose and twist with wrong orientation.
   - aftMappedTrans : geometry_msgs::msg::TransformStamped
       The TF message being broadcasted, containing the current pose with wrong orientation.
   - odomAftMappedAdjusted : nav_msgs::msg::Odometry
       The odometry message being published, containing the current pose and twist with right orientation.
   - aftMappedTransAdjusted : geometry_msgs::msg::TransformStamped
       The TF message being broadcasted, containing the current pose with right orientation.

   Returns
   -------
   This function primarily interacts with ROS topics to publish the current pose and does not return any value. 
   This broadcasting is essential for maintaining a consistent and updated representation of the robot's pose within the map, 
   which is crucial for navigation, planning, and further map optimizations.

   Publishers
   ----------
   - pubOdomAftMapped : Publishes the robot's odometry after map optimization.
   - tfBroadcaster : Broadcasts the transform (pose) of the robot in the map frame.
   - pubOdomAftMappedAdjusted: Publishes the robot's odometry after map optimization and correcting the orientation.
*/

  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion geoQuat;
  q.setRPY(transformAftMapped[0], transformAftMapped[2], transformAftMapped[1]);
  geoQuat = tf2::toMsg(q);

  odomAftMapped.header.stamp = timeLaserOdometry;
  odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
  odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
  odomAftMapped.pose.pose.orientation.z = geoQuat.x;
  odomAftMapped.pose.pose.orientation.w = geoQuat.w;
  odomAftMapped.pose.pose.position.x = transformAftMapped[3];
  odomAftMapped.pose.pose.position.y = transformAftMapped[4];
  odomAftMapped.pose.pose.position.z = transformAftMapped[5];
  odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
  odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
  odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
  odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
  odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
  odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
  pubOdomAftMapped->publish(odomAftMapped);

  aftMappedTrans.header.stamp = timeLaserOdometry;
  aftMappedTrans.transform.translation.x = transformAftMapped[3];
  aftMappedTrans.transform.translation.y = transformAftMapped[4];
  aftMappedTrans.transform.translation.z = transformAftMapped[5];
  aftMappedTrans.transform.rotation.x = -geoQuat.y;
  aftMappedTrans.transform.rotation.y = -geoQuat.z;
  aftMappedTrans.transform.rotation.z = geoQuat.x;
  aftMappedTrans.transform.rotation.w = geoQuat.w;
  tfBroadcaster->sendTransform(aftMappedTrans);

  odomAftMappedAdjusted.header.stamp = timeLaserOdometry;
  odomAftMappedAdjusted.pose.pose.orientation.x = -geoQuat.x;
  odomAftMappedAdjusted.pose.pose.orientation.y = -geoQuat.y;
  odomAftMappedAdjusted.pose.pose.orientation.z = geoQuat.z;
  odomAftMappedAdjusted.pose.pose.orientation.w = geoQuat.w;
  odomAftMappedAdjusted.pose.pose.position.x = transformAftMapped[5];
  odomAftMappedAdjusted.pose.pose.position.y = transformAftMapped[3];
  odomAftMappedAdjusted.pose.pose.position.z = transformAftMapped[4];
  odomAftMappedAdjusted.twist.twist.angular.x = transformBefMapped[0];
  odomAftMappedAdjusted.twist.twist.angular.y = transformBefMapped[1];
  odomAftMappedAdjusted.twist.twist.angular.z = transformBefMapped[2];
  odomAftMappedAdjusted.twist.twist.linear.x = transformBefMapped[3];
  odomAftMappedAdjusted.twist.twist.linear.y = transformBefMapped[4];
  odomAftMappedAdjusted.twist.twist.linear.z = transformBefMapped[5];
  pubOdomAftMappedAdjusted->publish(odomAftMappedAdjusted);
  
  aftMappedTransAdjusted.header.stamp = timeLaserOdometry;
  aftMappedTransAdjusted.transform.translation.x = transformAftMapped[3];
  aftMappedTransAdjusted.transform.translation.y = transformAftMapped[4];
  aftMappedTransAdjusted.transform.translation.z = transformAftMapped[5];
  aftMappedTransAdjusted.transform.rotation.x = -geoQuat.x;
  aftMappedTransAdjusted.transform.rotation.y = -geoQuat.y;
  aftMappedTransAdjusted.transform.rotation.z = geoQuat.z;
  aftMappedTransAdjusted.transform.rotation.w = geoQuat.w;
  tfBroadcaster->sendTransform(aftMappedTransAdjusted);
}

void MapOptimization::publishKeyPosesAndFrames() {

  /* Summary: Publishes key poses and recent key frames as PointCloud2 messages if there are subscribers.

   Extended Description: This function checks for active subscribers to the `pubKeyPoses` and `pubRecentKeyFrames` topics. 
   If subscribers are present, it converts the stored key poses (`cloudKeyPoses3D`) and the downsampled surface points from 
   the most recent key frames (`laserCloudSurfFromMapDS`) into ROS PointCloud2 messages and publishes them. 
   These publications allow external entities, such as visualization tools or other ROS nodes, to access the latest key poses and key frames, 
   facilitating debugging, analysis, and further processing.

   Parameters
   ----------
   - cloudKeyPoses3D : pcl::PointCloud<PointType>::Ptr
       The current collection of key poses.
   - laserCloudSurfFromMapDS : pcl::PointCloud<PointType>::Ptr
       The downsampled surface points from the most recent key frames.
   - timeLaserOdometry : Time
       The timestamp for the latest odometry reading, used to stamp the published messages.
   - cloudMsgTemp : sensor_msgs::msg::PointCloud2
       Temporary variable for storing the converted PointCloud2 message before publishing.

   Returns
   -------
   This function's primary purpose is to publish data to ROS topics for external use. The ability to visualize key poses and frames
   in real-time is invaluable for monitoring the progress of SLAM operations and ensuring data integrity throughout the mapping process.

   Publishers
   -----------
   - pubKeyPoses : Publishes the current key poses as a PointCloud2 message.
   - pubRecentKeyFrames : Publishes the downsampled points from recent key frames as a PointCloud2 message.
*/

  if (pubKeyPoses->get_subscription_count() != 0) {
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry;
    cloudMsgTemp.header.frame_id = _cameraInit;
    pubKeyPoses->publish(cloudMsgTemp);
  }

  if (pubRecentKeyFrames->get_subscription_count() != 0) {
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry;
    cloudMsgTemp.header.frame_id = _cameraInit;
    pubRecentKeyFrames->publish(cloudMsgTemp);
  }
}

void MapOptimization::publishGlobalMap() {

  /* Summary: Publishes the global map to subscribers as a PointCloud2 message.

   Extended Description: This function is responsible for compiling the global map's key poses and corresponding key frames into
   a PointCloud2 message and publishing it, provided there are subscribers to the `pubLaserCloudSurround` topic. 
   It performs a radius search to find nearby key frames for visualization, downsamples them for efficiency, and combines corner, 
   surface, and outlier points from these key frames into a single point cloud for publication.

   Parameters
   ----------
   - cloudKeyPoses3D : pcl::PointCloud<PointType>::Ptr
       The current set of key poses in the global map.
   - globalMapKeyPoses, globalMapKeyPosesDS : pcl::PointCloud<PointType>::Ptr
       Temporary point clouds for storing the key poses to be visualized and their downsampled version.
   - globalMapKeyFrames, globalMapKeyFramesDS : pcl::PointCloud<PointType>::Ptr
       Point clouds aggregating the corner, surface, and outlier points from the key frames to be visualized.
   - pointSearchIndGlobalMap, pointSearchSqDisGlobalMap : std::vector<int>, std::vector<float>
       Lists used to store indices and squared distances of the key poses found in the radius search.
   - timeLaserOdometry : Time
       The timestamp for the latest odometry reading, used to stamp the published message.

   Returns
   -------
   The primary goal of this function is to publish the compiled global map for visualization or further processing by external entities.
   This publication is crucial for monitoring the SLAM process, allowing for real-time visualization of the map as it evolves, 
   and facilitating the detection and correction of mapping errors.

   Publishers
   -----------
   - pubLaserCloudSurround : Publishes the aggregated global map consisting of key poses and their associated key frames as a PointCloud2 message.
*/

  if (pubLaserCloudSurround->get_subscription_count() == 0) return;

  if (cloudKeyPoses3D->points.empty() == true) return;
  // kd-tree to find near key frames to visualize
  std::vector<int> pointSearchIndGlobalMap;
  std::vector<float> pointSearchSqDisGlobalMap;
  // search near key frames to visualize
  mtx.lock();
  kdtreeGlobalMap.setInputCloud(cloudKeyPoses3D);
  kdtreeGlobalMap.radiusSearch(
      currentRobotPosPoint, _global_map_visualization_search_radius,
      pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
  mtx.unlock();

  for (size_t i = 0; i < pointSearchIndGlobalMap.size(); ++i)
    globalMapKeyPoses->points.push_back(
        cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
  // downsample near selected key frames
  downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
  downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
  // extract visualized and downsampled key frames
  for (size_t i = 0; i < globalMapKeyPosesDS->points.size(); ++i) {
    int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
    *globalMapKeyFrames += *transformPointCloud(
        cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    *globalMapKeyFrames += *transformPointCloud(
        surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
    *globalMapKeyFrames +=
        *transformPointCloud(outlierCloudKeyFrames[thisKeyInd],
                             &cloudKeyPoses6D->points[thisKeyInd]);
  }
  // downsample visualized points
  downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
  downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

  sensor_msgs::msg::PointCloud2 cloudMsgTemp;
  pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
  cloudMsgTemp.header.stamp = timeLaserOdometry;
  cloudMsgTemp.header.frame_id = _cameraInit;
  pubLaserCloudSurround->publish(cloudMsgTemp);

  globalMapKeyPoses->clear();
  globalMapKeyPosesDS->clear();
  globalMapKeyFrames->clear();
  //globalMapKeyFramesDS->clear();
}

bool MapOptimization::detectLoopClosure() {

  /* Summary: Detects potential loop closures by finding historical key frames close to the current robot position.

   Extended Description: This method searches for historical key frames within a specified radius from the current robot position
   to detect potential loop closures. It identifies the closest historical key frame based on the time difference from the current time. 
   If a suitable candidate is found, it prepares the latest and historical key frames for loop closure detection by transforming and 
   downsampling their point clouds.

   Parameters
   ----------
   - latestSurfKeyFrameCloud, nearHistorySurfKeyFrameCloud, nearHistorySurfKeyFrameCloudDS : pcl::PointCloud<PointType>::Ptr
       Point clouds for storing the latest key frame, nearby historical key frames, and their downsampled versions.
   - pointSearchIndLoop, pointSearchSqDisLoop : std::vector<int>, std::vector<float>
       Vectors for storing the indices and squared distances of the historical key frames found within the search radius.
   - closestHistoryFrameID : int
       Index of the closest historical key frame suitable for loop closure detection.
   - timeLaserOdometry : Time
       The timestamp of the latest odometry measurement.

   Returns
   -------
   bool
       True if a potential loop closure is detected by finding a historical key frame close to the current robot position. False otherwise.

   This method is a critical component of the SLAM process, enabling the system to identify when the robot revisits a previously mapped area. 
   Detecting and correctly handling loop closures is essential for maintaining map consistency and accuracy over time.

   Publishers
   ----------
   - pubHistoryKeyFrames : Publishes the downsampled point cloud of nearby historical key frames for visualization or further processing.
*/

  latestSurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloud->clear();
  nearHistorySurfKeyFrameCloudDS->clear();

  std::lock_guard<std::mutex> lock(mtx);
  std::vector<int> pointSearchIndLoop;
  std::vector<float> pointSearchSqDisLoop;
  kdtreeHistoryKeyPoses.setInputCloud(cloudKeyPoses3D);
  kdtreeHistoryKeyPoses.radiusSearch(currentRobotPosPoint, _history_keyframe_search_radius, pointSearchIndLoop, pointSearchSqDisLoop);

  closestHistoryFrameID = -1;
  for (size_t i = 0; i < pointSearchIndLoop.size(); ++i) {
    int id = pointSearchIndLoop[i];
    if (abs(cloudKeyPoses6D->points[id].time - timeLaserOdometry.seconds()) > 30.0) {
      closestHistoryFrameID = id;
      break;
    }
  }
  if (closestHistoryFrameID == -1) {
    return false;
  }

  // Add check for the robot's current position close to (x = 0, y = 0)
  if (sqrt(currentRobotPosPoint.x * currentRobotPosPoint.x + currentRobotPosPoint.y * currentRobotPosPoint.y) > 1.0) {
    return false;
  }

  // Save latest key frames
  latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
  *latestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  *latestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);

  // Save history near key frames
  for (int j = - _history_keyframe_search_num; j <= _history_keyframe_search_num; ++j) {
    if (closestHistoryFrameID + j < 0 || closestHistoryFrameID + j > latestFrameIDLoopCloure)
      continue;
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[closestHistoryFrameID + j], &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
    *nearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[closestHistoryFrameID + j], &cloudKeyPoses6D->points[closestHistoryFrameID + j]);
  }

  downSizeFilterHistoryKeyFrames.setInputCloud(nearHistorySurfKeyFrameCloud);
  downSizeFilterHistoryKeyFrames.filter(*nearHistorySurfKeyFrameCloudDS);

  if (pubHistoryKeyFrames->get_subscription_count() != 0) {
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry;
    cloudMsgTemp.header.frame_id = _cameraInit;
    pubHistoryKeyFrames->publish(cloudMsgTemp);
  }

  return true;
}

void MapOptimization::performLoopClosure() {

  /* Summary: Performs loop closure when a potential loop is detected.

   Extended Description: This method attempts to perform loop closure by aligning the latest key frame with a previously visited area
   represented by a historical key frame. It uses Iterative Closest Point (ICP) for alignment and evaluates the fitness score to determine
   the quality of alignment. If the alignment is successful and meets the fitness criteria, it publishes the corrected cloud for visualization 
   and updates the global pose graph with the new constraint.

   Parameters
   ----------
   - latestSurfKeyFrameCloud : pcl::PointCloud<PointType>::Ptr
       The latest surface key frame cloud for loop closure.
   - nearHistorySurfKeyFrameCloudDS : pcl::PointCloud<PointType>::Ptr
       The downsampled historical surface key frame cloud used for ICP alignment.
   - icp : pcl::IterativeClosestPoint<PointType, PointType>
       The Iterative Closest Point algorithm instance configured for loop closure.
   - correctionCameraFrame, correctionLidarFrame : Eigen::Affine3f
       Transformations for aligning and correcting the pose of the latest key frame.
   - poseFrom, poseTo : gtsam::Pose3
       The poses used to add a new constraint to the pose graph for loop closure.

   Returns
   -------
   This method is crucial for correcting drift in the map by identifying and closing loops. Successful loop closure significantly improves 
   the accuracy and consistency of the map by ensuring that the global pose graph remains consistent with previously visited areas.

   Publishers
   ----------
   - pubIcpKeyFrames : Publishes the corrected key frame cloud after successful alignment and loop closure.
*/

  if (cloudKeyPoses3D->points.empty())
    return;

  if (potentialLoopFlag == false) {
    if (detectLoopClosure() == true) {
      potentialLoopFlag = true;
      RCLCPP_WARN(this->get_logger(), "Loop Closure is detected");
      
      std_msgs::msg::Bool loop_flag_msg;
      loop_flag_msg.data = true;
      pubLoopClosureStatus->publish(loop_flag_msg);
    }
    if (potentialLoopFlag == false) {
      std_msgs::msg::Bool loop_flag_msg;
      loop_flag_msg.data = false;
      pubLoopClosureStatus->publish(loop_flag_msg);
      return;
    }
  }

  // Reset the flag first no matter if ICP succeeds or not
  potentialLoopFlag = false;
  std_msgs::msg::Bool loop_flag_msg;
  loop_flag_msg.data = false;
  pubLoopClosureStatus->publish(loop_flag_msg);

  // ICP Settings
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(latestSurfKeyFrameCloud);
  icp.setInputTarget(nearHistorySurfKeyFrameCloudDS);
  pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
  icp.align(*unused_result);

  if (icp.hasConverged() == false || icp.getFitnessScore() > _history_keyframe_fitness_score)
    return;

  // Publish corrected cloud
  if (pubIcpKeyFrames->get_subscription_count() != 0) {
    pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
    sensor_msgs::msg::PointCloud2 cloudMsgTemp;
    pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
    cloudMsgTemp.header.stamp = timeLaserOdometry;
    cloudMsgTemp.header.frame_id = _cameraInit;
    pubIcpKeyFrames->publish(cloudMsgTemp);
  }

  float x, y, z, roll, pitch, yaw;
  Eigen::Matrix4f correctionMatrix = icp.getFinalTransformation();
  Eigen::Affine3f correctionCameraFrame(correctionMatrix);
  pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
  Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);

  Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
  Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
  pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
  gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
  gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[closestHistoryFrameID]);
  gtsam::Vector Vector6(6);
  float noiseScore = icp.getFitnessScore();
  Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
  auto constraintNoise = noiseModel::Diagonal::Variances(Vector6);

  std::lock_guard<std::mutex> lock(mtx);
  gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise));
  isam->update(gtSAMgraph);
  isam->update();
  gtSAMgraph.resize(0);

  aLoopIsClosed = true;
}

void MapOptimization::extractSurroundingKeyFrames() {

  /* Summary: Extracts surrounding key frames for localization and mapping.

   Extended Description: This method extracts key frames surrounding the current position for the purpose of scan-to-map optimization
   or loop closure. It distinguishes between operating modes based on whether loop closure is enabled. 
   In loop closure mode, it uses recent key frames; otherwise, it searches the entire set of key frames. 
   It manages the key frames by downsampling for efficiency and updates the local map with these key frames for subsequent processing steps.

   Parameters
   ----------
   - cloudKeyPoses3D : pcl::PointCloud<PointType>::Ptr
       A cloud containing the 3D positions of all key frames.
   - cloudKeyPoses6D : pcl::PointCloud<PointTypePose>::Ptr
       A cloud containing the 6D poses (position + orientation) of all key frames.
   - recentCornerCloudKeyFrames, recentSurfCloudKeyFrames, recentOutlierCloudKeyFrames : std::deque<pcl::PointCloud<PointType>::Ptr>
       Queues storing the most recent corner, surface, and outlier key frames for loop closure.
   - surroundingKeyPoses, surroundingKeyPosesDS : pcl::PointCloud<PointType>::Ptr
       Clouds storing the surrounding key poses and their downsampled versions for scan-to-map optimization.
   - kdtreeSurroundingKeyPoses : pcl::KdTreeFLANN<PointType>
       A KD-tree structure to efficiently find surrounding key poses.
   - downSizeFilterCorner, downSizeFilterSurf : pcl::VoxelGrid<PointType>
       Downsample filters for reducing the resolution of point clouds for performance optimization.

   Returns
   -------
   This method updates the local map with key frames surrounding the current position, optimizing computational efficiency and ensuring
   the relevance of the map for the current localization and mapping tasks.

   Note
   ----
   The method dynamically adjusts the set of key frames used for mapping based on the robot's movement and 
   the operational mode (loop closure enabled or not). This adaptive strategy is crucial for maintaining an 
   efficient and accurate map representation.
*/

  if (cloudKeyPoses3D->points.empty() == true) return;

  if (_loop_closure_enabled == true) {
    // only use recent key poses for graph building
    if (static_cast<int>(recentCornerCloudKeyFrames.size()) <
        _surrounding_keyframe_search_num) {  // queue is not full (the beginning
                                         // of mapping or a loop is just
                                         // closed)
                                         // clear recent key frames queue
      recentCornerCloudKeyFrames.clear();
      recentSurfCloudKeyFrames.clear();
      recentOutlierCloudKeyFrames.clear();
      int numPoses = cloudKeyPoses3D->points.size();
      for (int i = numPoses - 1; i >= 0; --i) {
        int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
        PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
        updateTransformPointCloudSinCos(&thisTransformation);
        // extract surrounding map
        recentCornerCloudKeyFrames.push_front(
            transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
        recentSurfCloudKeyFrames.push_front(
            transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
        recentOutlierCloudKeyFrames.push_front(
            transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
        if (static_cast<int>(recentCornerCloudKeyFrames.size()) >= _surrounding_keyframe_search_num)
          break;
      }
    } else {  // queue is full, pop the oldest key frame and push the latest
              // key frame
      if (latestFrameID != static_cast<int>(cloudKeyPoses3D->points.size()) - 1) {
        // if the robot is not moving, no need to
        // update recent frames

        recentCornerCloudKeyFrames.pop_front();
        recentSurfCloudKeyFrames.pop_front();
        recentOutlierCloudKeyFrames.pop_front();
        // push latest scan to the end of queue
        latestFrameID = cloudKeyPoses3D->points.size() - 1;
        PointTypePose thisTransformation =
            cloudKeyPoses6D->points[latestFrameID];
        updateTransformPointCloudSinCos(&thisTransformation);
        recentCornerCloudKeyFrames.push_back(
            transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
        recentSurfCloudKeyFrames.push_back(
            transformPointCloud(surfCloudKeyFrames[latestFrameID]));
        recentOutlierCloudKeyFrames.push_back(
            transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
      }
    }

    for (size_t i = 0; i < recentCornerCloudKeyFrames.size(); ++i) {
      *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
      *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
      *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i];
    }
  } else {
    surroundingKeyPoses->clear();
    surroundingKeyPosesDS->clear();
    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses.setInputCloud(cloudKeyPoses3D);
    kdtreeSurroundingKeyPoses.radiusSearch(
        currentRobotPosPoint, (double)_surrounding_keyframe_search_radius,
        pointSearchInd, pointSearchSqDis);

    for (size_t i = 0; i < pointSearchInd.size(); ++i){
      surroundingKeyPoses->points.push_back(
          cloudKeyPoses3D->points[pointSearchInd[i]]);
    }

    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

    // delete key frames that are not in surrounding region
    int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
    for (size_t i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
      bool existingFlag = false;
      for (int j = 0; j < numSurroundingPosesDS; ++j) {
        if (surroundingExistingKeyPosesID[i] ==
            (int)surroundingKeyPosesDS->points[j].intensity) {
          existingFlag = true;
          break;
        }
      }
      if (existingFlag == false) {
        surroundingExistingKeyPosesID.erase(
            surroundingExistingKeyPosesID.begin() + i);
        surroundingCornerCloudKeyFrames.erase(
            surroundingCornerCloudKeyFrames.begin() + i);
        surroundingSurfCloudKeyFrames.erase(
            surroundingSurfCloudKeyFrames.begin() + i);
        surroundingOutlierCloudKeyFrames.erase(
            surroundingOutlierCloudKeyFrames.begin() + i);
        --i;
      }
    }
    // add new key frames that are not in calculated existing key frames
    for (int i = 0; i < numSurroundingPosesDS; ++i) {
      bool existingFlag = false;
      for (auto iter = surroundingExistingKeyPosesID.begin();
           iter != surroundingExistingKeyPosesID.end(); ++iter) {
        if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity) {
          existingFlag = true;
          break;
        }
      }
      if (existingFlag == true) {
        continue;
      } else {
        int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
        PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
        updateTransformPointCloudSinCos(&thisTransformation);
        surroundingExistingKeyPosesID.push_back(thisKeyInd);
        surroundingCornerCloudKeyFrames.push_back(
            transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
        surroundingSurfCloudKeyFrames.push_back(
            transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
        surroundingOutlierCloudKeyFrames.push_back(
            transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
      }
    }

    for (size_t i = 0; i < surroundingExistingKeyPosesID.size(); ++i) {
      *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
      *laserCloudSurfFromMap += *surroundingSurfCloudKeyFrames[i];
      *laserCloudSurfFromMap += *surroundingOutlierCloudKeyFrames[i];
    }
  }
  // Downsample the surrounding corner key frames (or map)
  downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
  downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
  laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
  // Downsample the surrounding surf key frames (or map)
  downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
  downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
  laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
}

void MapOptimization::downsampleCurrentScan() {

  /* Summary: Downsamples the current scan's point clouds for efficient processing.

   Extended Description: This method applies voxel grid downsampling to the latest corner, surface, and outlier point clouds 
   from the LiDAR scan. It aims to reduce the density of the point clouds, thus decreasing computational load for subsequent 
   processing stages such as feature extraction, matching, and optimization. The method aggregates downsampled surface and 
   outlier clouds for a comprehensive representation of the scan.

   Parameters
   ----------
   - laserCloudCornerLastDS : pcl::PointCloud<PointType>::Ptr
       The downsampled corner point cloud.
   - laserCloudSurfLastDS : pcl::PointCloud<PointType>::Ptr
       The downsampled surface point cloud.
   - laserCloudOutlierLastDS : pcl::PointCloud<PointType>::Ptr
       The downsampled outlier point cloud.
   - laserCloudSurfTotalLast : pcl::PointCloud<PointType>::Ptr
       The combined surface and outlier point clouds before downsampling.
   - laserCloudSurfTotalLastDS : pcl::PointCloud<PointType>::Ptr
       The downsampled version of the combined surface and outlier point clouds.
   - downSizeFilterCorner : pcl::VoxelGrid<PointType>
       The downsampling filter for corner points.
   - downSizeFilterSurf : pcl::VoxelGrid<PointType>
       The downsampling filter for surface points.
   - downSizeFilterOutlier : pcl::VoxelGrid<PointType>
       The downsampling filter for outlier points.
   - laserCloudCornerLastDSNum : int
       The number of points in the downsampled corner point cloud.
   - laserCloudSurfLastDSNum : int
       The number of points in the downsampled surface point cloud.
   - laserCloudOutlierLastDSNum : int
       The number of points in the downsampled outlier point cloud.
   - laserCloudSurfTotalLastDSNum : int
       The number of points in the downsampled combined surface and outlier point clouds.

   Returns
   -------
   This method updates internal point cloud members to their downsampled versions, ready for further processing such as 
   feature extraction and map optimization.

   Note
   ----
   Downsampling is a critical step in maintaining real-time performance of the LiDAR odometry and mapping system by managing 
   the computational complexity associated with processing dense point clouds.
*/

  laserCloudCornerLastDS->clear();
  downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
  downSizeFilterCorner.filter(*laserCloudCornerLastDS);
  laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();

  laserCloudSurfLastDS->clear();
  downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
  downSizeFilterSurf.filter(*laserCloudSurfLastDS);
  laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();

  laserCloudOutlierLastDS->clear();
  downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
  downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
  laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

  laserCloudSurfTotalLast->clear();
  laserCloudSurfTotalLastDS->clear();
  *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
  *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
  downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
  downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
  laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
}

void MapOptimization::cornerOptimization() {

  /* Summary: Optimizes the corner features by matching them to the corner features of the map.

   Extended Description: This method finds the nearest neighbors of the corner points in the map and calculates 
   the line features for optimization. It uses principal component analysis (PCA) to estimate the line's direction and computes 
   the coefficients for the edge constraint. These coefficients are used in the optimization process to refine the pose of the 
   LiDAR scan within the map.

   Parameters
   ----------
   - pointOri : PointType
       The original point from the downsampled corner point cloud.
   - pointSel : PointType
       The point after being transformed to the map frame.
   - pointSearchInd : std::vector<int>
       Indices of the nearest corner points found in the map.
   - pointSearchSqDis : std::vector<float>
       Squared distances to the nearest corner points found in the map.
   - matA1, matD1, matV1 : Eigen::Matrix3f, Eigen::Vector3f, Eigen::Matrix3f
       Matrices for eigen decomposition in PCA.
   - coeff : PointType
       Coefficients of the plane fitted to the nearest points (for optimization).
   - laserCloudOri : pcl::PointCloud<PointType>::Ptr
       Points for which an edge constraint can be formed.
   - coeffSel : pcl::PointCloud<PointType>::Ptr
       Coefficients of the constraints for the optimizer.

   Returns
   -------
   This method updates `laserCloudOri` and `coeffSel` with points and their corresponding coefficients for optimization.

   Note
   ----
   This process contributes to the accurate alignment of the scan within the map by adjusting the pose of the LiDAR based 
   on the detected corner features, leveraging the geometric constraints provided by the environment's edges.
*/

  updatePointAssociateToMapSinCos();
  for (int i = 0; i < laserCloudCornerLastDSNum; i++) {
    pointOri = laserCloudCornerLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd,
                                       pointSearchSqDis);

    if (pointSearchSqDis[4] < 1.0) {
      float cx = 0, cy = 0, cz = 0;
      for (int j = 0; j < 5; j++) {
        cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
        cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
        cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
      }
      cx /= 5;
      cy /= 5;
      cz /= 5;

      float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
      for (int j = 0; j < 5; j++) {
        float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
        float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
        float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

        a11 += ax * ax;
        a12 += ax * ay;
        a13 += ax * az;
        a22 += ay * ay;
        a23 += ay * az;
        a33 += az * az;
      }
      a11 /= 5;
      a12 /= 5;
      a13 /= 5;
      a22 /= 5;
      a23 /= 5;
      a33 /= 5;

      matA1(0, 0) = a11;
      matA1(0, 1) = a12;
      matA1(0, 2) = a13;
      matA1(1, 0) = a12;
      matA1(1, 1) = a22;
      matA1(1, 2) = a23;
      matA1(2, 0) = a13;
      matA1(2, 1) = a23;
      matA1(2, 2) = a33;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);

      matD1 = esolver.eigenvalues().real();
      matV1 = esolver.eigenvectors().real();

      if (matD1[2] > 3 * matD1[1]) {
        float x0 = pointSel.x;
        float y0 = pointSel.y;
        float z0 = pointSel.z;
        float x1 = cx + 0.1 * matV1(0, 0);
        float y1 = cy + 0.1 * matV1(0, 1);
        float z1 = cz + 0.1 * matV1(0, 2);
        float x2 = cx - 0.1 * matV1(0, 0);
        float y2 = cy - 0.1 * matV1(0, 1);
        float z2 = cz - 0.1 * matV1(0, 2);

        float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                              ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                          ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                              ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                          ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                              ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

        float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                         (z1 - z2) * (z1 - z2));

        float la =
            ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
             (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
            a012 / l12;

        float lb =
            -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
              (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float lc =
            -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
              (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
            a012 / l12;

        float ld2 = a012 / l12;

        float s = 1 - 0.9 * fabs(ld2);

        coeff.x = s * la;
        coeff.y = s * lb;
        coeff.z = s * lc;
        coeff.intensity = s * ld2;

        if (s > 0.1) {
          laserCloudOri->push_back(pointOri);
          coeffSel->push_back(coeff);
        }
      }
    }
  }
}

void MapOptimization::surfOptimization() {

  /* Summary: Optimizes the surface features by matching them to the surface features of the map.

   Extended Description: This method finds the nearest neighbors of the surface points in the map and calculates the plane features
   for optimization. It uses a least squares plane fitting method to estimate the plane's parameters and computes the coefficients
   for the surface constraint. These coefficients are used in the optimization process to refine the pose of the LiDAR scan within the map.

   Parameters
   ----------
   - pointOri : PointType
       The original point from the downsampled surface point cloud.
   - pointSel : PointType
       The point after being transformed to the map frame.
   - pointSearchInd : std::vector<int>
       Indices of the nearest surface points found in the map.
   - pointSearchSqDis : std::vector<float>
       Squared distances to the nearest surface points found in the map.
   - matA0, matB0, matX0 : Eigen::MatrixXf, Eigen::VectorXf, Eigen::Vector3f
       Matrices for solving the plane equation.
   - pa, pb, pc, pd : float
       Parameters of the plane equation fitted to the nearest points.
   - ps : float
       Scale factor for normalizing the plane parameters.
   - planeValid : bool
       Flag indicating if the detected plane is valid based on a predefined threshold.
   - coeff : PointType
       Coefficients of the plane fitted to the nearest points (for optimization).
   - laserCloudOri : pcl::PointCloud<PointType>::Ptr
       Points for which a plane constraint can be formed.
   - coeffSel : pcl::PointCloud<PointType>::Ptr
       Coefficients of the constraints for the optimizer.

   Returns
   -------
   This function updates `laserCloudOri` and `coeffSel` with points and their corresponding coefficients for optimization.

   Note
   ----
   This process contributes to the accurate alignment of the scan within the map by adjusting the pose of the LiDAR based
   on the detected surface features, leveraging the geometric constraints provided by the environment's planes.
*/

  updatePointAssociateToMapSinCos();
  for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++) {
    pointOri = laserCloudSurfTotalLastDS->points[i];
    pointAssociateToMap(&pointOri, &pointSel);
    kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd,
                                      pointSearchSqDis);

    if (pointSearchSqDis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        matA0(j, 0) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
        matA0(j, 1) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
        matA0(j, 2) =
            laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
      }
      matX0 = matA0.colPivHouseholderQr().solve(matB0);

      float pa = matX0(0, 0);
      float pb = matX0(1, 0);
      float pc = matX0(2, 0);
      float pd = 1;

      float ps = sqrt(pa * pa + pb * pb + pc * pc);
      pa /= ps;
      pb /= ps;
      pc /= ps;
      pd /= ps;

      bool planeValid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                 pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                 pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z +
                 pd) > 0.2) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {
        float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

        float s = 1 - 0.9 * fabs(pd2) /
                          sqrt(sqrt(pointSel.x * pointSel.x +
                                    pointSel.y * pointSel.y +
                                    pointSel.z * pointSel.z));

        coeff.x = s * pa;
        coeff.y = s * pb;
        coeff.z = s * pc;
        coeff.intensity = s * pd2;

        if (s > 0.1) {
          laserCloudOri->push_back(pointOri);
          coeffSel->push_back(coeff);
        }
      }
    }
  }
}

bool MapOptimization::LMOptimization(int iterCount) {

  /* Summary: Performs Levenberg-Marquardt optimization to refine the pose estimation of a LiDAR scan within the map.

   Extended Description: This function iteratively adjusts the pose of the current LiDAR scan (`transformTobeMapped`) 
   to align it more accurately with the map. It utilizes the Levenberg-Marquardt algorithm, solving for the pose that 
   minimizes the residual error between the observed scan points and the corresponding map points. The optimization process 
   repeats until the change in pose is below a predefined threshold or the maximum number of iterations is reached.

   Parameters
   ----------
   - iterCount : int
       The current iteration count within the optimization loop.

   - srx, crx, sry, cry, srz, crz : float
       Sine and cosine of the rotation angles from the current pose estimation, used for point transformation.
   - matA : Eigen::Matrix<float, Eigen::Dynamic, 6>
       The Jacobian matrix of the error function with respect to the pose parameters.
   - matAt, matAtA : Eigen::Matrix<float, 6, Eigen::Dynamic>, Eigen::Matrix<float, 6, 6>
       Transpose of matA and the product of matAt and matA, respectively, used in solving the normal equations.
   - matB : Eigen::VectorXf
       The error vector representing the difference between the current scan points and the map.
   - matX : Eigen::Matrix<float, 6, 1>
       The solution vector containing the incremental updates to the pose.
   - isDegenerate : bool
       Flag indicating if the system is degenerate and special handling is required.
   - matP : Eigen::Matrix<float, 6, 6>
       The projection matrix used to handle degenerate cases by projecting the solution to a stable subspace.
   - deltaR, deltaT : float
       The magnitude of rotation and translation updates in the current iteration, used for convergence check.

   Returns
   -------
   bool
       Indicates whether the optimization successfully converged (true) or not (false).

   This function updates `transformTobeMapped` with the optimized pose, including both orientation (rotation angles) 
   and position (translation), to align the current LiDAR scan more accurately with the map.
*/

  float srx = sin(transformTobeMapped[0]);
  float crx = cos(transformTobeMapped[0]);
  float sry = sin(transformTobeMapped[1]);
  float cry = cos(transformTobeMapped[1]);
  float srz = sin(transformTobeMapped[2]);
  float crz = cos(transformTobeMapped[2]);

  int laserCloudSelNum = laserCloudOri->points.size();
  if (laserCloudSelNum < 50) {
    return false;
  }

  Eigen::Matrix<float,Eigen::Dynamic,6> matA(laserCloudSelNum, 6);
  Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,laserCloudSelNum);
  Eigen::Matrix<float,6,6> matAtA;
  Eigen::VectorXf matB(laserCloudSelNum);
  Eigen::Matrix<float,6,1> matAtB;
  Eigen::Matrix<float,6,1> matX;

  for (int i = 0; i < laserCloudSelNum; i++) {
    pointOri = laserCloudOri->points[i];
    coeff = coeffSel->points[i];

    float arx =
        (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y -
         srx * sry * pointOri.z) *
            coeff.x +
        (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) *
            coeff.y +
        (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y -
         cry * srx * pointOri.z) *
            coeff.z;

    float ary =
        ((cry * srx * srz - crz * sry) * pointOri.x +
         (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) *
            coeff.x +
        ((-cry * crz - srx * sry * srz) * pointOri.x +
         (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) *
            coeff.z;

    float arz = ((crz * srx * sry - cry * srz) * pointOri.x +
                 (-cry * crz - srx * sry * srz) * pointOri.y) *
                    coeff.x +
                (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y +
                ((sry * srz + cry * crz * srx) * pointOri.x +
                 (crz * sry - cry * srx * srz) * pointOri.y) *
                    coeff.z;

    matA(i, 0) = arx;
    matA(i, 1) = ary;
    matA(i, 2) = arz;
    matA(i, 3) = coeff.x;
    matA(i, 4) = coeff.y;
    matA(i, 5) = coeff.z;
    matB(i, 0) = -coeff.intensity;
  }
  matAt = matA.transpose();
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  matX = matAtA.colPivHouseholderQr().solve(matAtB);

  if (iterCount == 0) {
    Eigen::Matrix<float,1,6> matE;
    Eigen::Matrix<float,6,6> matV;
    Eigen::Matrix<float,6,6> matV2;

    Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
    matE = esolver.eigenvalues().real();
    matV = esolver.eigenvectors().real();

     matV2 = matV;

    isDegenerate = false;
    float eignThre[6] = {100, 100, 100, 100, 100, 100};
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
    Eigen::Matrix<float,6, 1> matX2(matX);
    matX2 = matX;
    matX = matP * matX2;
  }

  transformTobeMapped[0] += matX(0, 0);
  transformTobeMapped[1] += matX(1, 0);
  transformTobeMapped[2] += matX(2, 0);
  transformTobeMapped[3] += matX(3, 0);
  transformTobeMapped[4] += matX(4, 0);
  transformTobeMapped[5] += matX(5, 0);

  float deltaR = sqrt(pow(pcl::rad2deg(matX(0, 0)), 2) +
                      pow(pcl::rad2deg(matX(1, 0)), 2) +
                      pow(pcl::rad2deg(matX(2, 0)), 2));
  float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                      pow(matX(4, 0) * 100, 2) +
                      pow(matX(5, 0) * 100, 2));

  if (deltaR < 0.05 && deltaT < 0.05) {
    return true;
  }
  return false;
}

void MapOptimization::scan2MapOptimization() {

  /* Summary: Executes the scan-to-map optimization process to refine the pose of the current LiDAR scan with respect to the map.

   Extended Description: This method aligns the current LiDAR scan to the map by iteratively performing corner and surface optimizations, 
   followed by a Levenberg-Marquardt optimization step. It leverages the pre-built KD-trees of map points (both corner and surface) to find 
   correspondences, then optimizes the pose to reduce the alignment error. The process iterates up to a maximum number of times or until 
   convergence.

   Parameters
   ----------
   - kdtreeCornerFromMap, kdtreeSurfFromMap : pcl::KdTreeFLANN<PointType>
       KD-trees for efficiently searching corner and surface points in the map.
   - iterCount : int
       Counter for the optimization iterations within this scan-to-map optimization session.
   - laserCloudOri, coeffSel : pcl::PointCloud<PointType>::Ptr
       Point clouds for storing points involved in the optimization and their corresponding coefficients.

   Returns
   -------
   This function updates the global pose estimation (`transformTobeMapped`) of the current LiDAR scan by aligning it more accurately 
   to the pre-built map. This is achieved through a series of corner and surface optimizations, followed by a Levenberg-Marquardt optimization
   step to refine the pose based on the alignment error between the scan and the map.
*/

  if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100) {
    kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMapDS);
    kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMapDS);

    for (int iterCount = 0; iterCount < 10; iterCount++) {
      laserCloudOri->clear();
      coeffSel->clear();

      cornerOptimization();
      surfOptimization();

      if (LMOptimization(iterCount) == true) break;
    }

    transformUpdate();
  }
}

void MapOptimization::saveKeyFramesAndFactor() {

  /* Summary: Saves the current key frame and factors into the pose graph for later optimization.

   Extended Description: This function evaluates if the current scan should be saved as a key frame based on the movement from 
   the last key frame. If the movement exceeds a certain threshold, it updates the pose graph with the new key frame and 
   the transformation factor between the last and current key frames. It utilizes gtsam for graph optimization to maintain 
   a globally consistent map.

   Parameters
   ----------
   - currentRobotPosPoint : PointType
       The current position of the robot in the map.
   - previousRobotPosPoint : PointType
       The previous position of the robot for comparison to determine if a new key frame should be saved.
   - Vector6 : gtsam::Vector
       The noise model for the pose graph factors.
   - priorNoise, odometryNoise : gtsam::noiseModel::Diagonal
       The noise models for the prior and odometry factors in the pose graph.
   - saveThisKeyFrame : bool
       A flag to determine if the current frame should be saved based on movement threshold.
   - thisPose3D, thisPose6D : PointType, PointTypePose
       The 3D and 6D representations of the robot's pose to be saved.
   - latestEstimate : gtsam::Pose3
       The latest pose estimate from the pose graph optimization.
   - thisCornerKeyFrame, thisSurfKeyFrame, thisOutlierKeyFrame : pcl::PointCloud<PointType>::Ptr
       Point clouds for the current key frame's corner points, surface points, and outliers.

   Returns
   -------
   The updates the pose graph with the new key frame and the transformation factor between the last and current key frames. 
   Saves the 3D and 6D pose of the key frame, along with the associated corner, surface, and outlier point clouds for later use. 
   Also updates the global pose transformation (`transformAftMapped`) based on the latest pose graph optimization.
*/

  currentRobotPosPoint.x = transformAftMapped[3];
  currentRobotPosPoint.y = transformAftMapped[4];
  currentRobotPosPoint.z = transformAftMapped[5];

  gtsam::Vector Vector6(6);
  Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  auto priorNoise = noiseModel::Diagonal::Variances(Vector6);
  auto odometryNoise = noiseModel::Diagonal::Variances(Vector6);

  bool saveThisKeyFrame = true;
  if (sqrt((previousRobotPosPoint.x - currentRobotPosPoint.x) *
               (previousRobotPosPoint.x - currentRobotPosPoint.x) +
           (previousRobotPosPoint.y - currentRobotPosPoint.y) *
               (previousRobotPosPoint.y - currentRobotPosPoint.y) +
           (previousRobotPosPoint.z - currentRobotPosPoint.z) *
               (previousRobotPosPoint.z - currentRobotPosPoint.z)) < 0.3) {
    saveThisKeyFrame = false;
  }

  if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty()) return;

  previousRobotPosPoint = currentRobotPosPoint;
  /**
   * update grsam graph
   */
  if (cloudKeyPoses3D->points.empty()) {
    gtSAMgraph.add(PriorFactor<Pose3>(
        0,
        Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                           transformTobeMapped[1]),
              Point3(transformTobeMapped[5], transformTobeMapped[3],
                     transformTobeMapped[4])),
        priorNoise));
    initialEstimate.insert(
        0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0],
                              transformTobeMapped[1]),
                 Point3(transformTobeMapped[5], transformTobeMapped[3],
                        transformTobeMapped[4])));
    for (int i = 0; i < 6; ++i) transformLast[i] = transformTobeMapped[i];
  } else {
    gtsam::Pose3 poseFrom = Pose3(
        Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
        Point3(transformLast[5], transformLast[3], transformLast[4]));
    gtsam::Pose3 poseTo =
        Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4]));
    gtSAMgraph.add(BetweenFactor<Pose3>(
        cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(),
        poseFrom.between(poseTo), odometryNoise));
    initialEstimate.insert(
        cloudKeyPoses3D->points.size(),
        Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0],
                           transformAftMapped[1]),
              Point3(transformAftMapped[5], transformAftMapped[3],
                     transformAftMapped[4])));
  }
  /**
   * update iSAM
   */
  isam->update(gtSAMgraph, initialEstimate);
  isam->update();

  gtSAMgraph.resize(0);
  initialEstimate.clear();

  /**
   * save key poses
   */
  PointType thisPose3D;
  PointTypePose thisPose6D;
  Pose3 latestEstimate;

  isamCurrentEstimate = isam->calculateEstimate();
  latestEstimate =
      isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

  thisPose3D.x = latestEstimate.translation().y();
  thisPose3D.y = latestEstimate.translation().z();
  thisPose3D.z = latestEstimate.translation().x();
  thisPose3D.intensity =
      cloudKeyPoses3D->points.size();  // this can be used as index
  cloudKeyPoses3D->push_back(thisPose3D);

  thisPose6D.x = thisPose3D.x;
  thisPose6D.y = thisPose3D.y;
  thisPose6D.z = thisPose3D.z;
  thisPose6D.intensity = thisPose3D.intensity;  // this can be used as index
  thisPose6D.roll = latestEstimate.rotation().pitch();
  thisPose6D.pitch = latestEstimate.rotation().yaw();
  thisPose6D.yaw = latestEstimate.rotation().roll();  // in camera frame
  thisPose6D.time = timeLaserOdometry.seconds();
  cloudKeyPoses6D->push_back(thisPose6D);
  /**
   * save updated transform
   */
  if (cloudKeyPoses3D->points.size() > 1) {
    transformAftMapped[0] = latestEstimate.rotation().pitch();
    transformAftMapped[1] = latestEstimate.rotation().yaw();
    transformAftMapped[2] = latestEstimate.rotation().roll();
    transformAftMapped[3] = latestEstimate.translation().y();
    transformAftMapped[4] = latestEstimate.translation().z();
    transformAftMapped[5] = latestEstimate.translation().x();

    for (int i = 0; i < 6; ++i) {
      transformLast[i] = transformAftMapped[i];
      transformTobeMapped[i] = transformAftMapped[i];
    }
  }

  pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(
      new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(
      new pcl::PointCloud<PointType>());

  pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
  pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
  pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

  cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
  surfCloudKeyFrames.push_back(thisSurfKeyFrame);
  outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
}

void MapOptimization::correctPoses() {

  /* Summary: Corrects the poses of key frames after a loop closure.

   Extended Description: This function is triggered after a loop closure is detected and the pose graph optimization is performed. 
   It updates the poses of all key frames in the map to reflect the optimized poses. The poses are corrected based on the latest 
   estimates from the pose graph optimizer, ensuring global consistency of the map.

   Parameters
   ----------
   - aLoopIsClosed : bool
       A flag indicating whether a loop closure has been detected and processed.
   - numPoses : int
       The number of poses in the pose graph after optimization.
   - isamCurrentEstimate : gtsam::Values
       The current set of optimized poses from the pose graph optimization.

   Returns
   -------
   This function updates the 3D and 6D poses of all key frames in the map to reflect the optimized poses from the pose graph. 
   This ensures that the map remains globally consistent after loop closures. Resets the loop closure flag to prepare for 
   future detections.
*/

  if (aLoopIsClosed == true) {
    recentCornerCloudKeyFrames.clear();
    recentSurfCloudKeyFrames.clear();
    recentOutlierCloudKeyFrames.clear();
    // update key poses
    int numPoses = isamCurrentEstimate.size();
    for (int i = 0; i < numPoses; ++i) {
      cloudKeyPoses3D->points[i].x =
          isamCurrentEstimate.at<Pose3>(i).translation().y();
      cloudKeyPoses3D->points[i].y =
          isamCurrentEstimate.at<Pose3>(i).translation().z();
      cloudKeyPoses3D->points[i].z =
          isamCurrentEstimate.at<Pose3>(i).translation().x();

      cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
      cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
      cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
      cloudKeyPoses6D->points[i].roll =
          isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
      cloudKeyPoses6D->points[i].pitch =
          isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
      cloudKeyPoses6D->points[i].yaw =
          isamCurrentEstimate.at<Pose3>(i).rotation().roll();
    }

    aLoopIsClosed = false;
  }
}

void MapOptimization::clearCloud() {

  /* Summary: Clears the temporary clouds used for map optimization.

   Extended Description: This function clears the point clouds that store corner and surface features extracted from 
   the map for the purpose of scan-to-map optimization. It's typically called after the optimization process to free up 
   memory and prepare for the next cycle of data processing.

   Parameters
   ----------
   - laserCloudCornerFromMap : pcl::PointCloud<PointType>::Ptr
       The point cloud storing corner features extracted from the map.
   - laserCloudSurfFromMap : pcl::PointCloud<PointType>::Ptr
       The point cloud storing surface features extracted from the map.
   - laserCloudCornerFromMapDS : pcl::PointCloud<PointType>::Ptr
       The downsampled point cloud of corner features for faster processing.
   - laserCloudSurfFromMapDS : pcl::PointCloud<PointType>::Ptr
       The downsampled point cloud of surface features for faster processing.

   Returns
   -------
   This function clears the aforementioned point clouds to ensure that the memory is not unnecessarily occupied between successive 
   iterations of the map optimization process. This helps in maintaining efficient use of resources during the SLAM process.
*/

  laserCloudCornerFromMap->clear();
  laserCloudSurfFromMap->clear();
  laserCloudCornerFromMapDS->clear();
  laserCloudSurfFromMapDS->clear();
}


void MapOptimization::run() {

  /* Summary: The main loop of the MapOptimization node, responsible for processing incoming lidar scans and optimizing the map.

   Extended Description: This function continuously receives lidar scan data, performs scan-to-map optimization, and updates the map with 
   new keyframes. It orchestrates the workflow of map optimization, including extracting surrounding key frames, downsampling current scans, 
   optimizing the current scan against the map, saving key frames and factors for pose graph optimization, correcting poses after loop closure,
   and managing the publication of transformations and key poses. It also triggers loop closure detection and global 
   map publication at specified intervals.

   Parameters
   ----------
   - cycle_count : size_t
       Counter for the number of cycles the run loop has executed.
   - association : AssociationOut
       Structure containing the latest lidar scan data and associated information.
   - transformSum : float[6]
       Array holding the cumulative odometry transformations.
   - mtx : std::mutex
       Mutex for synchronizing access to shared resources.

   Returns
   -------
   This function p rocesses incoming lidar scans to optimize and update the map. Saves new key frames and manages the pose graph for SLAM. 
   Corrects the map poses following loop closures and manages the periodic publication of the global map and transformation data.
*/

  // Publish ready status after initialization
  statusPublisher.ready();

  size_t cycle_count = 0;

  while (rclcpp::ok()) {
    AssociationOut association;
    _input_channel.receive(association);
    if( !rclcpp::ok() ) break;

    {
      std::lock_guard<std::mutex> lock(mtx);

      laserCloudCornerLast = association.cloud_corner_last;
      laserCloudSurfLast = association.cloud_surf_last;
      laserCloudOutlierLast = association.cloud_outlier_last;

      timeLaserOdometry = association.laser_odometry.header.stamp;

      OdometryToTransform(association.laser_odometry, transformSum);

      transformAssociateToMap();

      extractSurroundingKeyFrames();

      downsampleCurrentScan();

      scan2MapOptimization();

      saveKeyFramesAndFactor();

      correctPoses();

      publishTF();

      publishKeyPosesAndFrames();

      clearCloud();
    }

    // Publish running status
    statusPublisher.running();

    cycle_count++;

    if ((cycle_count % 3) == 0) {
      _loop_closure_signal.send(true);
    }

    if ((cycle_count % 10) == 0) {
      _publish_global_signal.send(true);
    }
  }
}
