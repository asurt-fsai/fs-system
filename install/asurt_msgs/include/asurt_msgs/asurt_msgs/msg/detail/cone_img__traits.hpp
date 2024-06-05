// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asurt_msgs:msg/ConeImg.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__CONE_IMG__TRAITS_HPP_
#define ASURT_MSGS__MSG__DETAIL__CONE_IMG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asurt_msgs/msg/detail/cone_img__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'img'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace asurt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ConeImg & msg,
  std::ostream & out)
{
  out << "{";
  // member: detection_id
  {
    out << "detection_id: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_id, out);
    out << ", ";
  }

  // member: rows
  {
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << ", ";
  }

  // member: cols
  {
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
    out << ", ";
  }

  // member: img
  {
    out << "img: ";
    to_flow_style_yaml(msg.img, out);
    out << ", ";
  }

  // member: track_id
  {
    out << "track_id: ";
    rosidl_generator_traits::value_to_yaml(msg.track_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConeImg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: detection_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_id: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_id, out);
    out << "\n";
  }

  // member: rows
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rows: ";
    rosidl_generator_traits::value_to_yaml(msg.rows, out);
    out << "\n";
  }

  // member: cols
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cols: ";
    rosidl_generator_traits::value_to_yaml(msg.cols, out);
    out << "\n";
  }

  // member: img
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "img:\n";
    to_block_style_yaml(msg.img, out, indentation + 2);
  }

  // member: track_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "track_id: ";
    rosidl_generator_traits::value_to_yaml(msg.track_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConeImg & msg, bool use_flow_style = false)
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

}  // namespace asurt_msgs

namespace rosidl_generator_traits
{

[[deprecated("use asurt_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const asurt_msgs::msg::ConeImg & msg,
  std::ostream & out, size_t indentation = 0)
{
  asurt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asurt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const asurt_msgs::msg::ConeImg & msg)
{
  return asurt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asurt_msgs::msg::ConeImg>()
{
  return "asurt_msgs::msg::ConeImg";
}

template<>
inline const char * name<asurt_msgs::msg::ConeImg>()
{
  return "asurt_msgs/msg/ConeImg";
}

template<>
struct has_fixed_size<asurt_msgs::msg::ConeImg>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value> {};

template<>
struct has_bounded_size<asurt_msgs::msg::ConeImg>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value> {};

template<>
struct is_message<asurt_msgs::msg::ConeImg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASURT_MSGS__MSG__DETAIL__CONE_IMG__TRAITS_HPP_
