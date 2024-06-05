// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asurt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
#define ASURT_MSGS__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asurt_msgs/msg/detail/node_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace asurt_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const NodeStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NodeStatus & msg,
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

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NodeStatus & msg, bool use_flow_style = false)
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
  const asurt_msgs::msg::NodeStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  asurt_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asurt_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const asurt_msgs::msg::NodeStatus & msg)
{
  return asurt_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asurt_msgs::msg::NodeStatus>()
{
  return "asurt_msgs::msg::NodeStatus";
}

template<>
inline const char * name<asurt_msgs::msg::NodeStatus>()
{
  return "asurt_msgs/msg/NodeStatus";
}

template<>
struct has_fixed_size<asurt_msgs::msg::NodeStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<asurt_msgs::msg::NodeStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<asurt_msgs::msg::NodeStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASURT_MSGS__MSG__DETAIL__NODE_STATUS__TRAITS_HPP_
