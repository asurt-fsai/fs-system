// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asurt_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__BOUNDING_BOXES__TRAITS_HPP_
#define ASURT_MSGS__MSG__DETAIL__BOUNDING_BOXES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asurt_msgs/msg/detail/bounding_boxes__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'bounding_boxes'
#include "asurt_msgs/msg/detail/bounding_box__traits.hpp"

namespace asurt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BoundingBoxes & msg,
  std::ostream & out)
{
  out << "{";
  // member: view_id
  {
    out << "view_id: ";
    rosidl_generator_traits::value_to_yaml(msg.view_id, out);
    out << ", ";
  }

  // member: object_count
  {
    out << "object_count: ";
    rosidl_generator_traits::value_to_yaml(msg.object_count, out);
    out << ", ";
  }

  // member: bounding_boxes
  {
    if (msg.bounding_boxes.size() == 0) {
      out << "bounding_boxes: []";
    } else {
      out << "bounding_boxes: [";
      size_t pending_items = msg.bounding_boxes.size();
      for (auto item : msg.bounding_boxes) {
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
  const BoundingBoxes & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: view_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "view_id: ";
    rosidl_generator_traits::value_to_yaml(msg.view_id, out);
    out << "\n";
  }

  // member: object_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_count: ";
    rosidl_generator_traits::value_to_yaml(msg.object_count, out);
    out << "\n";
  }

  // member: bounding_boxes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bounding_boxes.size() == 0) {
      out << "bounding_boxes: []\n";
    } else {
      out << "bounding_boxes:\n";
      for (auto item : msg.bounding_boxes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BoundingBoxes & msg, bool use_flow_style = false)
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
  const asurt_msgs::msg::BoundingBoxes & msg,
  std::ostream & out, size_t indentation = 0)
{
  asurt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asurt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const asurt_msgs::msg::BoundingBoxes & msg)
{
  return asurt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asurt_msgs::msg::BoundingBoxes>()
{
  return "asurt_msgs::msg::BoundingBoxes";
}

template<>
inline const char * name<asurt_msgs::msg::BoundingBoxes>()
{
  return "asurt_msgs/msg/BoundingBoxes";
}

template<>
struct has_fixed_size<asurt_msgs::msg::BoundingBoxes>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<asurt_msgs::msg::BoundingBoxes>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<asurt_msgs::msg::BoundingBoxes>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASURT_MSGS__MSG__DETAIL__BOUNDING_BOXES__TRAITS_HPP_
