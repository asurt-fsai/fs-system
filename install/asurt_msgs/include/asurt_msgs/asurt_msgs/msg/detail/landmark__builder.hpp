// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_Landmark_probability
{
public:
  explicit Init_Landmark_probability(::asurt_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::Landmark probability(::asurt_msgs::msg::Landmark::_probability_type arg)
  {
    msg_.probability = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::Landmark msg_;
};

class Init_Landmark_identifier
{
public:
  explicit Init_Landmark_identifier(::asurt_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_probability identifier(::asurt_msgs::msg::Landmark::_identifier_type arg)
  {
    msg_.identifier = std::move(arg);
    return Init_Landmark_probability(msg_);
  }

private:
  ::asurt_msgs::msg::Landmark msg_;
};

class Init_Landmark_type
{
public:
  explicit Init_Landmark_type(::asurt_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  Init_Landmark_identifier type(::asurt_msgs::msg::Landmark::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Landmark_identifier(msg_);
  }

private:
  ::asurt_msgs::msg::Landmark msg_;
};

class Init_Landmark_position
{
public:
  Init_Landmark_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Landmark_type position(::asurt_msgs::msg::Landmark::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Landmark_type(msg_);
  }

private:
  ::asurt_msgs::msg::Landmark msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::Landmark>()
{
  return asurt_msgs::msg::builder::Init_Landmark_position();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
