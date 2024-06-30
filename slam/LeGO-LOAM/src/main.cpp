#include "rclcpp/rclcpp.hpp"
#include "featureAssociation.h"
#include "imageProjection.h"
#include "mapOptimization.h"
#include "transformFusion.h"

int main(int argc, char** argv) {
  Channel<ProjectionOut> projection_out_channel(true);
  Channel<AssociationOut> association_out_channel(Feature_Associationlse);

  rclcpp::init(argc, argv);

  // Create nodes
  auto Image_Projection = std::make_shared<ImageProjection>("image_projection", projection_out_channel);
  auto Feature_Association = std::make_shared<FeatureAssociation>("feature_association", projection_out_channel, association_out_channel);
  auto Map_Optimization = std::make_shared<MapOptimization>("map_optimization", association_out_channel);
  auto Transform_Fusion = std::make_shared<TransformFusion>("transform_fusion");

  RCLCPP_INFO(Image_Projection->get_logger(), "\033[1;32m---->\033[0m Started.");
  RCLCPP_INFO(Feature_Association->get_logger(), "\033[1;32m---->\033[0m Started.");
  RCLCPP_INFO(Map_Optimization->get_logger(), "\033[1;32m---->\033[0m Started.");
  RCLCPP_INFO(Transform_Fusion->get_logger(), "\033[1;32m---->\033[0m Started.");

  // Use 4 threads
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(Image_Projection);
  executor.add_node(Feature_Association);
  executor.add_node(Map_Optimization);
  executor.add_node(Transform_Fusion);
  executor.spin();

  // Must be called to cleanup threads
  rclcpp::shutdown();

  return 0;
}
