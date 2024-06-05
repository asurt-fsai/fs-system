// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asurt_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__TRAITS_HPP_
#define ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asurt_msgs/msg/detail/landmark_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'landmarks'
#include "asurt_msgs/msg/detail/landmark__traits.hpp"

namespace asurt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LandmarkArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: landmarks
  {
    if (msg.landmarks.size() == 0) {
      out << "landmarks: []";
    } else {
      out << "landmarks: [";
      size_t pending_items = msg.landmarks.size();
      for (auto item : msg.landmarks) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LandmarkArray & msg,
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

  // member: landmarks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.landmarks.size() == 0) {
      out << "landmarks: []\n";
    } else {
      out << "landmarks:\n";
      for (auto item : msg.landmarks) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LandmarkArray & msg, bool use_flow_style = false)
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
  const asurt_msgs::msg::LandmarkArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  asurt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asurt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const asurt_msgs::msg::LandmarkArray & msg)
{
  return asurt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asurt_msgs::msg::LandmarkArray>()
{
  return "asurt_msgs::msg::LandmarkArray";
}

template<>
inline const char * name<asurt_msgs::msg::LandmarkArray>()
{
  return "asurt_msgs/msg/LandmarkArray";
}

template<>
struct has_fixed_size<asurt_msgs::msg::LandmarkArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<asurt_msgs::msg::LandmarkArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<asurt_msgs::msg::LandmarkArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__TRAITS_HPP_
