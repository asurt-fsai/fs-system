// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asurt_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__KEY_POINTS__TRAITS_HPP_
#define ASURT_MSGS__MSG__DETAIL__KEY_POINTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asurt_msgs/msg/detail/key_points__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace asurt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const KeyPoints & msg,
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

  // member: track_ids
  {
    if (msg.track_ids.size() == 0) {
      out << "track_ids: []";
    } else {
      out << "track_ids: [";
      size_t pending_items = msg.track_ids.size();
      for (auto item : msg.track_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: classes
  {
    if (msg.classes.size() == 0) {
      out << "classes: []";
    } else {
      out << "classes: [";
      size_t pending_items = msg.classes.size();
      for (auto item : msg.classes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: keypoints
  {
    if (msg.keypoints.size() == 0) {
      out << "keypoints: []";
    } else {
      out << "keypoints: [";
      size_t pending_items = msg.keypoints.size();
      for (auto item : msg.keypoints) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const KeyPoints & msg,
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

  // member: track_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.track_ids.size() == 0) {
      out << "track_ids: []\n";
    } else {
      out << "track_ids:\n";
      for (auto item : msg.track_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: classes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.classes.size() == 0) {
      out << "classes: []\n";
    } else {
      out << "classes:\n";
      for (auto item : msg.classes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: keypoints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.keypoints.size() == 0) {
      out << "keypoints: []\n";
    } else {
      out << "keypoints:\n";
      for (auto item : msg.keypoints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const KeyPoints & msg, bool use_flow_style = false)
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
  const asurt_msgs::msg::KeyPoints & msg,
  std::ostream & out, size_t indentation = 0)
{
  asurt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asurt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const asurt_msgs::msg::KeyPoints & msg)
{
  return asurt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asurt_msgs::msg::KeyPoints>()
{
  return "asurt_msgs::msg::KeyPoints";
}

template<>
inline const char * name<asurt_msgs::msg::KeyPoints>()
{
  return "asurt_msgs/msg/KeyPoints";
}

template<>
struct has_fixed_size<asurt_msgs::msg::KeyPoints>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<asurt_msgs::msg::KeyPoints>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<asurt_msgs::msg::KeyPoints>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASURT_MSGS__MSG__DETAIL__KEY_POINTS__TRAITS_HPP_
