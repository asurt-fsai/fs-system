#include "ros/ros.h"
#include "../include/tree_search.h"
#include "asurt_msgs/LandmarkArray.h"
#include "asurt_msgs/Landmark.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Path.h"

ros::Publisher path_pub;
ros::Publisher possible_waypoints_pub;
std::string frame_id;
TreeSearchParams params;
CostParams cost_params;

void conesCallback(const asurt_msgs::LandmarkArray& cones_msg){
    std::vector<Cone> cones;
    for (int i = 0; i < cones_msg.landmarks.size(); i++) {
        cones.emplace_back(cones_msg.landmarks[i].position.x,
                            cones_msg.landmarks[i].position.y,
                            cones_msg.landmarks[i].type);
    }
    TreeSearch tree_search = TreeSearch(cones, params, cost_params);
    std::tuple<Path, std::vector<Path>> output = tree_search.getPath();

    nav_msgs::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = ros::Time::now();
    for (int i = 0; i < std::get<0>(output).waypoints.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = std::get<0>(output).waypoints[i].x;
        pose.pose.position.y = std::get<0>(output).waypoints[i].y;
        path.poses.push_back(pose);
    }
    path_pub.publish(path);

    // Publish tree_search.waypoints as a list of points for visualization (MarkerArray)
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(tree_search.waypoints.size()+1);
    marker_array.markers[0].header.frame_id = frame_id;
    marker_array.markers[0].header.stamp = ros::Time::now();
    marker_array.markers[0].ns = "delete";
    marker_array.markers[0].type = visualization_msgs::Marker::SPHERE;
    marker_array.markers[0].action = visualization_msgs::Marker::DELETEALL;
    for(int i = 0; i < tree_search.waypoints.size(); i++){
        marker_array.markers[i+1].header.frame_id = frame_id;
        marker_array.markers[i+1].header.stamp = ros::Time::now();
        marker_array.markers[i+1].ns = "waypoints";
        marker_array.markers[i+1].id = i;
        marker_array.markers[i+1].type = visualization_msgs::Marker::SPHERE;
        marker_array.markers[i+1].action = visualization_msgs::Marker::ADD;
        marker_array.markers[i+1].pose.position.x = tree_search.waypoints[i][0];
        marker_array.markers[i+1].pose.position.y = tree_search.waypoints[i][1];
        marker_array.markers[i+1].pose.position.z = 0;
        marker_array.markers[i+1].pose.orientation.x = 0.0;
        marker_array.markers[i+1].pose.orientation.y = 0.0;
        marker_array.markers[i+1].pose.orientation.z = 0.0;
        marker_array.markers[i+1].pose.orientation.w = 1.0;
        marker_array.markers[i+1].scale.x = 0.1;
        marker_array.markers[i+1].scale.y = 0.1;
        marker_array.markers[i+1].scale.z = 0.1;
        marker_array.markers[i+1].color.a = 1.0;
        marker_array.markers[i+1].color.r = 0.0;
        marker_array.markers[i+1].color.g = 1.0;
        marker_array.markers[i+1].color.b = 0.0;
    }
    possible_waypoints_pub.publish(marker_array);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_trees_cpp");
  ros::NodeHandle nh;
  std::string cones_topic;
  std::string cpp_out_cones_topic;
  nh.getParam("/planning/cpp_cones_topic", cones_topic);
  nh.getParam("/planning/cpp_out_cones_topic", cpp_out_cones_topic);
  nh.getParam("/planning/frame_id", frame_id);

  // Planner parameters, cost params
  nh.getParam("/physical/track_width", cost_params.track_width);
  nh.getParam("/planning/track_width_weight", cost_params.track_width_weight);
  nh.getParam("/planning/color_weight", cost_params.color_weight);
  nh.getParam("/planning/angle_weight", cost_params.angle_weight);
  nh.getParam("/planning/error_no_detect", cost_params.error_no_detect);
  nh.getParam("/planning/field_of_view", params.field_of_view);
  nh.getParam("/planning/distance", params.distance);
  nh.getParam("/planning/max_search_iterations", params.max_search_iterations);
  nh.getParam("/planning/max_waypoints_per_path", params.max_waypoints_per_path);
  nh.getParam("/planning/waypoint_field_of_view", params.waypoint_field_of_view);
  nh.getParam("/planning/waypoint_distance", params.waypoint_distance);
  nh.getParam("/planning/path_queue_limit", params.path_queue_limit);
  nh.getParam("/planning/triangulation_radius", params.triangulation_radius);
  nh.getParam("/planning/triangulation_min_cone_dist", params.triangulation_min_cone_dist);
  nh.getParam("/planning/triangulation_min_waypoint_dist", params.triangulation_min_waypoint_dist);


  path_pub = nh.advertise<nav_msgs::Path>(cpp_out_cones_topic, 1);
  possible_waypoints_pub = nh.advertise<visualization_msgs::MarkerArray>("possible_waypoints", 1);

  ros::Subscriber cones_sub = nh.subscribe(cones_topic, 1, conesCallback);
  ros::spin();
  return 0;
}
