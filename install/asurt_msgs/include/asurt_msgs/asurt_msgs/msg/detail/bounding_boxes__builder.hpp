// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/bounding_boxes__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_BoundingBoxes_bounding_boxes
{
public:
  explicit Init_BoundingBoxes_bounding_boxes(::asurt_msgs::msg::BoundingBoxes & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::BoundingBoxes bounding_boxes(::asurt_msgs::msg::BoundingBoxes::_bounding_boxes_type arg)
  {
    msg_.bounding_boxes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBoxes msg_;
};

class Init_BoundingBoxes_object_count
{
public:
  explicit Init_BoundingBoxes_object_count(::asurt_msgs::msg::BoundingBoxes & msg)
  : msg_(msg)
  {}
  Init_BoundingBoxes_bounding_boxes object_count(::asurt_msgs::msg::BoundingBoxes::_object_count_type arg)
  {
    msg_.object_count = std::move(arg);
    return Init_BoundingBoxes_bounding_boxes(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBoxes msg_;
};

class Init_BoundingBoxes_view_id
{
public:
  Init_BoundingBoxes_view_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBoxes_object_count view_id(::asurt_msgs::msg::BoundingBoxes::_view_id_type arg)
  {
    msg_.view_id = std::move(arg);
    return Init_BoundingBoxes_object_count(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBoxes msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::BoundingBoxes>()
{
  return asurt_msgs::msg::builder::Init_BoundingBoxes_view_id();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__BOUNDING_BOXES__BUILDER_HPP_
