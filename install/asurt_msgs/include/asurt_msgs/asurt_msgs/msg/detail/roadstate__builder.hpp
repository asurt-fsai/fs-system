// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/Roadstate.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__ROADSTATE__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__ROADSTATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/roadstate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_Roadstate_header
{
public:
  explicit Init_Roadstate_header(::asurt_msgs::msg::Roadstate & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::Roadstate header(::asurt_msgs::msg::Roadstate::_header_type arg)
  {
    msg_.header = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::Roadstate msg_;
};

class Init_Roadstate_distance
{
public:
  explicit Init_Roadstate_distance(::asurt_msgs::msg::Roadstate & msg)
  : msg_(msg)
  {}
  Init_Roadstate_header distance(::asurt_msgs::msg::Roadstate::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_Roadstate_header(msg_);
  }

private:
  ::asurt_msgs::msg::Roadstate msg_;
};

class Init_Roadstate_laps
{
public:
  Init_Roadstate_laps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Roadstate_distance laps(::asurt_msgs::msg::Roadstate::_laps_type arg)
  {
    msg_.laps = std::move(arg);
    return Init_Roadstate_distance(msg_);
  }

private:
  ::asurt_msgs::msg::Roadstate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::Roadstate>()
{
  return asurt_msgs::msg::builder::Init_Roadstate_laps();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__ROADSTATE__BUILDER_HPP_
