// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/node_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_NodeStatus_message
{
public:
  explicit Init_NodeStatus_message(::asurt_msgs::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::NodeStatus message(::asurt_msgs::msg::NodeStatus::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::NodeStatus msg_;
};

class Init_NodeStatus_status
{
public:
  explicit Init_NodeStatus_status(::asurt_msgs::msg::NodeStatus & msg)
  : msg_(msg)
  {}
  Init_NodeStatus_message status(::asurt_msgs::msg::NodeStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_NodeStatus_message(msg_);
  }

private:
  ::asurt_msgs::msg::NodeStatus msg_;
};

class Init_NodeStatus_header
{
public:
  Init_NodeStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NodeStatus_status header(::asurt_msgs::msg::NodeStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NodeStatus_status(msg_);
  }

private:
  ::asurt_msgs::msg::NodeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::NodeStatus>()
{
  return asurt_msgs::msg::builder::Init_NodeStatus_header();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__NODE_STATUS__BUILDER_HPP_
