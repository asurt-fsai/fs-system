#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include "StatusPublisher.h"


class FeatureAssociation : public rclcpp::Node {

 public:
  FeatureAssociation(const std::string &name, Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);

  ~FeatureAssociation();

  void runFeatureAssociation();

 private:

  int verticalScans;
  int horizontalScans;
  float scanPeriod;
  float edgeThreshold;
  float surfThreshold;
  float nearestFeatureDistSqr;
  int mappingFrequencyDiv;

  std::string _laserCloudSharp;
  std::string _laserCloudLessSharp;
  std::string _laserCloudFlat;
  std::string _laserCloudLessFlat;
  std::string _laserCloudCornerLast;
  std::string _laserCloudSurfLast;
  std::string _outlierCloudLast;
  std::string _laserOdometry;

  std::string _camera;
  std::string _cameraInit;
  std::string _laserOdom;

  StatusPublisher statusPublisher;

  std::thread _run_thread;

  Channel<ProjectionOut>& _input_channel;
  Channel<AssociationOut>& _output_channel;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsLessSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

  pcl::VoxelGrid<PointType> downSizeFilter;

  asurt_msgs::msg::CloudInfo segInfo;
  std_msgs::msg::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudCornerLast;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudSurfLast;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOutlierCloudLast;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::msg::Odometry laserOdometry;

  geometry_msgs::msg::TransformStamped laserOdometryTrans;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  
  bool isDegenerate;

  int frameCount;
  size_t _cycle_count;

 private:
  void initializationValue();
  void adjustDistortion();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);

  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);

  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);

  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void checkSystemInitialization();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometry();

  void adjustOutlierCloud();
  void publishCloudsLast();

};

#endif // FEATUREASSOCIATION_H
