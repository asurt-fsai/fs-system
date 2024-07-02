#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include <Eigen/QR>
#include <StatusPublisher.h>

class ImageProjection : public rclcpp::Node {
 public:

  ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel);

  ~ImageProjection() = default;

  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

 private:
  void findStartEndAngle();
  void resetParameters();
  void projectPointCloud();
  void groundRemoval();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void publishClouds();

  StatusPublisher statusPublisher;


  pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

  pcl::PointCloud<PointType>::Ptr _full_cloud;
  pcl::PointCloud<PointType>::Ptr _full_info_cloud;

  pcl::PointCloud<PointType>::Ptr _ground_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
  pcl::PointCloud<PointType>::Ptr _outlier_cloud;

  int verticalScans;
  int horizontalScans;
  float angBottom;
  float angResolutionX;
  float angResolutionY;
  float segmentTheta;
  int segmentValidPointNum;
  int segmentValidLineNum;
  int groundScanIndex;
  float sensorMountAngle;

  std::string _lidarPoints;
  std::string _fullCloudProjected;
  std::string _fullCloudInfo;
  std::string _groundCloud;
  std::string _segmentedCloud;
  std::string _segmentedCloudPure;
  std::string _segmentedCloudInfo;
  std::string _outlierCloud;
  std::string _baseLink;

  Channel<ProjectionOut>& _output_channel;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFullCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFullInfoCloud;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGroundCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSegmentedCloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSegmentedCloudPure;
  rclcpp::Publisher<asurt_msgs::msg::CloudInfo>::SharedPtr pubSegmentedCloudInfo;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOutlierCloud;

  asurt_msgs::msg::CloudInfo _seg_msg;

  int _label_count;

  Eigen::MatrixXf _range_mat;   // range matrix for range image
  Eigen::MatrixXi _label_mat;   // label matrix for segmentaiton marking
  Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  // ground matrix for ground cloud marking


};



#endif  // IMAGEPROJECTION_H
