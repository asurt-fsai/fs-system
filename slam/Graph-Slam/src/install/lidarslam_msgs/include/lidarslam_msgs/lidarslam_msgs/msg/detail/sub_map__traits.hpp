// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidarslam_msgs:msg/SubMap.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__TRAITS_HPP_
#define LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lidarslam_msgs/msg/detail/sub_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'cloud'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace lidarslam_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SubMap & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: cloud
  {
    out << "cloud: ";
    to_flow_style_yaml(msg.cloud, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SubMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: cloud
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud:\n";
    to_block_style_yaml(msg.cloud, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SubMap & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace lidarslam_msgs

namespace rosidl_generator_traits
{

[[deprecated("use lidarslam_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const lidarslam_msgs::msg::SubMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  lidarslam_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lidarslam_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const lidarslam_msgs::msg::SubMap & msg)
{
  return lidarslam_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lidarslam_msgs::msg::SubMap>()
{
  return "lidarslam_msgs::msg::SubMap";
}

template<>
inline const char * name<lidarslam_msgs::msg::SubMap>()
{
  return "lidarslam_msgs/msg/SubMap";
}

template<>
struct has_fixed_size<lidarslam_msgs::msg::SubMap>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<sensor_msgs::msg::PointCloud2>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<lidarslam_msgs::msg::SubMap>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<sensor_msgs::msg::PointCloud2>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<lidarslam_msgs::msg::SubMap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__TRAITS_HPP_
