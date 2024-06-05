// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__KEY_POINTS__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__KEY_POINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/key_points__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_KeyPoints_keypoints
{
public:
  explicit Init_KeyPoints_keypoints(::asurt_msgs::msg::KeyPoints & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::KeyPoints keypoints(::asurt_msgs::msg::KeyPoints::_keypoints_type arg)
  {
    msg_.keypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::KeyPoints msg_;
};

class Init_KeyPoints_classes
{
public:
  explicit Init_KeyPoints_classes(::asurt_msgs::msg::KeyPoints & msg)
  : msg_(msg)
  {}
  Init_KeyPoints_keypoints classes(::asurt_msgs::msg::KeyPoints::_classes_type arg)
  {
    msg_.classes = std::move(arg);
    return Init_KeyPoints_keypoints(msg_);
  }

private:
  ::asurt_msgs::msg::KeyPoints msg_;
};

class Init_KeyPoints_track_ids
{
public:
  explicit Init_KeyPoints_track_ids(::asurt_msgs::msg::KeyPoints & msg)
  : msg_(msg)
  {}
  Init_KeyPoints_classes track_ids(::asurt_msgs::msg::KeyPoints::_track_ids_type arg)
  {
    msg_.track_ids = std::move(arg);
    return Init_KeyPoints_classes(msg_);
  }

private:
  ::asurt_msgs::msg::KeyPoints msg_;
};

class Init_KeyPoints_object_count
{
public:
  explicit Init_KeyPoints_object_count(::asurt_msgs::msg::KeyPoints & msg)
  : msg_(msg)
  {}
  Init_KeyPoints_track_ids object_count(::asurt_msgs::msg::KeyPoints::_object_count_type arg)
  {
    msg_.object_count = std::move(arg);
    return Init_KeyPoints_track_ids(msg_);
  }

private:
  ::asurt_msgs::msg::KeyPoints msg_;
};

class Init_KeyPoints_view_id
{
public:
  Init_KeyPoints_view_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyPoints_object_count view_id(::asurt_msgs::msg::KeyPoints::_view_id_type arg)
  {
    msg_.view_id = std::move(arg);
    return Init_KeyPoints_object_count(msg_);
  }

private:
  ::asurt_msgs::msg::KeyPoints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::KeyPoints>()
{
  return asurt_msgs::msg::builder::Init_KeyPoints_view_id();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__KEY_POINTS__BUILDER_HPP_
