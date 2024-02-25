// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidarslam_msgs:msg/MapArray.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__TRAITS_HPP_
#define LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "lidarslam_msgs/msg/detail/map_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'submaps'
#include "lidarslam_msgs/msg/detail/sub_map__traits.hpp"

namespace lidarslam_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MapArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: submaps
  {
    if (msg.submaps.size() == 0) {
      out << "submaps: []";
    } else {
      out << "submaps: [";
      size_t pending_items = msg.submaps.size();
      for (auto item : msg.submaps) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: cloud_coordinate
  {
    out << "cloud_coordinate: ";
    rosidl_generator_traits::value_to_yaml(msg.cloud_coordinate, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MapArray & msg,
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

  // member: submaps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.submaps.size() == 0) {
      out << "submaps: []\n";
    } else {
      out << "submaps:\n";
      for (auto item : msg.submaps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: cloud_coordinate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cloud_coordinate: ";
    rosidl_generator_traits::value_to_yaml(msg.cloud_coordinate, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MapArray & msg, bool use_flow_style = false)
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
  const lidarslam_msgs::msg::MapArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  lidarslam_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use lidarslam_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const lidarslam_msgs::msg::MapArray & msg)
{
  return lidarslam_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<lidarslam_msgs::msg::MapArray>()
{
  return "lidarslam_msgs::msg::MapArray";
}

template<>
inline const char * name<lidarslam_msgs::msg::MapArray>()
{
  return "lidarslam_msgs/msg/MapArray";
}

template<>
struct has_fixed_size<lidarslam_msgs::msg::MapArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<lidarslam_msgs::msg::MapArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<lidarslam_msgs::msg::MapArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__TRAITS_HPP_
