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

#include "transformFusion.h"
#include "StatusPublisher.h"

TransformFusion::TransformFusion(const std::string &name) : Node(name), statusPublisher("/status/transformFusion", this) {

  /* Constructor for the TransformFusion class, initializing the ROS node, publishers, and subscribers for odometry data integration.

   This constructor sets up the TransformFusion node to subscribe to odometry data (`/laser_odom_to_init`) and 
   mapped odometry data (`/aft_mapped_to_init`). It publishes integrated odometry data to the `/integrated_to_init` topic. 
   The constructor initializes various transformation arrays used for processing the odometry and mapping data, ensuring all 
   transformation values start at zero. It also sets up a TransformBroadcaster for sending out the final, fused transformation to other ROS nodes.

   Parameters
   ----------
   name : std::string
       The name of the ROS node.

   Attributes
   ----------
   pubLaserOdometry2 : rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
       Publisher for the integrated odometry data.
   subLaserOdometry : rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
       Subscriber for the raw laser odometry data.
   subOdomAftMapped : rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
       Subscriber for the odometry data after map optimization.
   laserOdometry2 : nav_msgs::msg::Odometry
       The message for the integrated odometry to be published, with frame IDs set for the 'camera_init' and 'Lidar'.
   laserOdometryTrans : geometry_msgs::msg::TransformStamped
       The transform message for broadcasting the odometry, with frame IDs set for the 'camera_init' and 'Lidar'.
   tfBroadcaster : std::shared_ptr<tf2_ros::TransformBroadcaster>
       Broadcaster for sending the final transformation to the tf2 transform library in ROS.
   transformSum : float[6]
       Array for accumulating transformations.
   transformIncre : float[6]
       Array for incremental transformations.
   transformMapped : float[6]
       Array for transformations after mapping.
   transformBefMapped : float[6]
       Array for transformations before mapping.
   transformAftMapped : float[6]
       Array for transformations after mapping adjustments.

   Methods
   -------
   void transformAssociateToMap():
       Associates the current laser odometry data with the map frame, adjusting the transformation to account for any discrepancies between the laser odometry and the map-optimized odometry.

   void laserOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr laserOdometry):
       Callback for the laser odometry subscriber. It processes the incoming laser odometry data, associates it with the map, and publishes and broadcasts the fused odometry.

       Parameters:
       - laserOdometry : nav_msgs::msg::Odometry::SharedPtr
           The incoming laser odometry data.

   void odomAftMappedHandler(const nav_msgs::msg::Odometry::SharedPtr odomAftMapped):
       Callback for the map-optimized odometry subscriber. It updates the internal state with the latest map-optimized odometry data, ensuring that the transformation to the map is accurate.

       Parameters:
       - odomAftMapped : nav_msgs::msg::Odometry::SharedPtr
           The incoming map-optimized odometry data.
*/

  // Publish starting status
  statusPublisher.starting();

  pubLaserOdometry2 = this->create_publisher<nav_msgs::msg::Odometry>("/integrated_to_init", 5);
  subLaserOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      "/laser_odom_to_init", 5, std::bind(&TransformFusion::laserOdometryHandler, this, std::placeholders::_1));
  subOdomAftMapped = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 5, std::bind(&TransformFusion::odomAftMappedHandler, this, std::placeholders::_1));

  laserOdometry2.header.frame_id = "camera_init";
  laserOdometry2.child_frame_id = "camera";

  laserOdometryTrans.header.frame_id = "camera_init";
  laserOdometryTrans.child_frame_id = "camera";

  tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  for (int i = 0; i < 6; ++i) {
    transformSum[i] = 0;
    transformIncre[i] = 0;
    transformMapped[i] = 0;
    transformBefMapped[i] = 0;
    transformAftMapped[i] = 0;
  }

  // Publish ready status
  statusPublisher.ready();
}

void TransformFusion::transformAssociateToMap() {

  /* Summary: Adjusts the transformation parameters to better align the current odometry with the map frame.

   Extended Description: This method computes incremental transformations by comparing the current odometry data (before mapping) 
   with the map-optimized data (after mapping). It uses trigonometric operations to adjust the pose of the vehicle, ensuring a 
   better alignment with the map. The adjustments involve calculating differences in position and orientation, then applying these 
   to refine the vehicle's estimated pose. This process aids in achieving a more accurate localization by ensuring the odometry data 
   is well aligned with the global map.

   Parameters
   ----------
   transformSum : float[6]
       Accumulated transformations up to the current point, used as a reference for adjustment calculations.
   transformBefMapped : float[6]
       Transformations before mapping, representing the vehicle's pose before map optimizations are applied.
   transformAftMapped : float[6]
       Transformations after mapping, representing the optimized pose post-map alignment.
   x1, y1, z1 : float
       Intermediate variables for calculating adjustments in the vehicle's position.
   x2, y2, z2 : float
       Adjusted position variables after applying rotational transformations.
   sbcx, cbcx, sbcy, cbcy, sbcz, cbcz : float
       Sine and cosine of the accumulated transformations, used for rotational adjustment calculations.
   sblx, cblx, sbly, cbly, sblz, cblz : float
       Sine and cosine of the transformations before mapping, used in pose adjustment calculations.
   salx, calx, saly, caly, salz, calz : float
       Sine and cosine of the transformations after mapping, for refining the pose adjustments.
   srx, srycrx, crycrx, srzcrx, crzcrx : float
       Variables for calculating the final rotational adjustments based on the alignment discrepancies.
   transformIncre : float[6]
       Incremental transformations computed to refine alignment with the map.
   transformMapped : float[6]
       The vehicle's pose estimation updated to reflect a more accurate alignment with the map.

   Returns
   -------
   This function updates internal state variables (transformIncre and transformMapped) to adjust the vehicle's pose based on 
   the current odometry data and its relation to the map.
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
  transformMapped[0] = -asin(srx);

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
  transformMapped[1] =
      atan2(srycrx / cos(transformMapped[0]), crycrx / cos(transformMapped[0]));

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
  transformMapped[2] =
      atan2(srzcrx / cos(transformMapped[0]), crzcrx / cos(transformMapped[0]));

  x1 = cos(transformMapped[2]) * transformIncre[3] -
       sin(transformMapped[2]) * transformIncre[4];
  y1 = sin(transformMapped[2]) * transformIncre[3] +
       cos(transformMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
  z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

  transformMapped[3] = transformAftMapped[3] - (cos(transformMapped[1]) * x2 +
                                                sin(transformMapped[1]) * z2);
  transformMapped[4] = transformAftMapped[4] - y2;
  transformMapped[5] = transformAftMapped[5] - (-sin(transformMapped[1]) * x2 +
                                                cos(transformMapped[1]) * z2);
}

void TransformFusion::laserOdometryHandler(
    const nav_msgs::msg::Odometry::SharedPtr laserOdometry) {

  /* Summary: Processes incoming laser odometry data, aligns it with the map, and publishes the integrated odometry.

   Extended Description: This method acts as a callback for incoming laser odometry messages. It converts the odometry 
   message to a transform format, stored in `transformSum`. It then adjusts this transform to align better with the map 
   using `transformAssociateToMap`. The adjusted transform updates the orientation and position in the `laserOdometry2` 
   message, which is then published. Additionally, the transform is broadcasted to other system components requiring the 
   vehicle's pose.

   Parameters
   ----------
   laserOdometry : const nav_msgs::msg::Odometry::SharedPtr
       Shared pointer to the incoming laser odometry message.

   q : tf2::Quaternion
       Quaternion for the rotation part of the transformed odometry data.
   geoQuat : geometry_msgs::msg::Quaternion
       ROS-compatible quaternion `q`, for message publishing.
   transformSum : float[6]
       Temporary transform format storage of incoming odometry data.
   transformMapped : float[6]
       Adjusted transform after map alignment.

   laserOdometry2 : nav_msgs::msg::Odometry
       Updated odometry message with pose based on the transformed and map-aligned data.
   laserOdometryTrans : geometry_msgs::msg::TransformStamped
       Transform message with updated translation and rotation for broadcasting.

   Publishers
   ----------
   pubLaserOdometry2 : rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
       Publishes the integrated odometry data to the `/integrated_to_init` topic.

   Returns
   -------
   This function publishes the `laserOdometry2` message and broadcasts the `laserOdometryTrans` transform.
*/

  statusPublisher.running();

  OdometryToTransform(*laserOdometry, transformSum);

  transformAssociateToMap();

  tf2::Quaternion q;
  geometry_msgs::msg::Quaternion geoQuat;
  q.setRPY(transformMapped[2], -transformMapped[0], -transformMapped[1]);
  geoQuat = tf2::toMsg(q);

  laserOdometry2.header.stamp = laserOdometry->header.stamp;
  laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
  laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
  laserOdometry2.pose.pose.orientation.z = geoQuat.x;
  laserOdometry2.pose.pose.orientation.w = geoQuat.w;
  laserOdometry2.pose.pose.position.x = transformMapped[3];
  laserOdometry2.pose.pose.position.y = transformMapped[4];
  laserOdometry2.pose.pose.position.z = transformMapped[5];
  pubLaserOdometry2->publish(laserOdometry2);

  laserOdometryTrans.header.stamp = laserOdometry->header.stamp;
  laserOdometryTrans.transform.translation.x = transformMapped[3];
  laserOdometryTrans.transform.translation.y = transformMapped[4];
  laserOdometryTrans.transform.translation.z = transformMapped[5];
  laserOdometryTrans.transform.rotation.x = -geoQuat.y;
  laserOdometryTrans.transform.rotation.y = -geoQuat.z;
  laserOdometryTrans.transform.rotation.z = geoQuat.x;
  laserOdometryTrans.transform.rotation.w = geoQuat.w;
  tfBroadcaster->sendTransform(laserOdometryTrans);
}

void TransformFusion::odomAftMappedHandler(
    const nav_msgs::msg::Odometry::SharedPtr odomAftMapped) {

  /* Summary: Updates transformations based on map-optimized odometry data.

   Extended Description: This callback function is invoked upon receiving map-optimized odometry data. It extracts the orientation 
   and position from the odometry message and converts the orientation from quaternion to roll, pitch, and yaw format. These values 
   are used to update the `transformAftMapped` array, reflecting the latest map-optimized pose. Additionally, the function captures 
   the vehicle's angular and linear velocities from the odometry message's twist component, updating the `transformBefMapped` array 
   to aid in subsequent transformations.

   Parameters
   ----------
   odomAftMapped : const nav_msgs::msg::Odometry::SharedPtr
       Shared pointer to the incoming map-optimized odometry message.

   roll, pitch, yaw : double
       Orientation of the vehicle extracted from the odometry message, converted from quaternion to Euler angles.
   geoQuat : geometry_msgs::msg::Quaternion
       The quaternion representing the vehicle's orientation, extracted directly from the odometry message.
   transformAftMapped : float[6]
       Updates this array with the latest pose (orientation and position) based on the map-optimized odometry data.
   transformBefMapped : float[6]
       Updates this array with the latest angular and linear velocities from the map-optimized odometry data.

   Subscribers
   -----------
   subOdomAftMapped : rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
       Subscribes to the `/aft_mapped_to_init` topic, receiving map-optimized odometry data that triggers this callback.

   Returns
   -------
   This function updates internal state variables to reflect the latest map-optimized pose and velocities.
*/

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  transformAftMapped[0] = -pitch;
  transformAftMapped[1] = -yaw;
  transformAftMapped[2] = roll;

  transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

  transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
  transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
  transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

  transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
  transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
  transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
}
