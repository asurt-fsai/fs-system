// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/bounding_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_BoundingBox_type
{
public:
  explicit Init_BoundingBox_type(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::BoundingBox type(::asurt_msgs::msg::BoundingBox::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_track_id
{
public:
  explicit Init_BoundingBox_track_id(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_type track_id(::asurt_msgs::msg::BoundingBox::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return Init_BoundingBox_type(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_detection_id
{
public:
  explicit Init_BoundingBox_detection_id(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_track_id detection_id(::asurt_msgs::msg::BoundingBox::_detection_id_type arg)
  {
    msg_.detection_id = std::move(arg);
    return Init_BoundingBox_track_id(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_height
{
public:
  explicit Init_BoundingBox_height(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_detection_id height(::asurt_msgs::msg::BoundingBox::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_BoundingBox_detection_id(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_width
{
public:
  explicit Init_BoundingBox_width(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_height width(::asurt_msgs::msg::BoundingBox::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_BoundingBox_height(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_y_center
{
public:
  explicit Init_BoundingBox_y_center(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_width y_center(::asurt_msgs::msg::BoundingBox::_y_center_type arg)
  {
    msg_.y_center = std::move(arg);
    return Init_BoundingBox_width(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_x_center
{
public:
  explicit Init_BoundingBox_x_center(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_y_center x_center(::asurt_msgs::msg::BoundingBox::_x_center_type arg)
  {
    msg_.x_center = std::move(arg);
    return Init_BoundingBox_y_center(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_ymax
{
public:
  explicit Init_BoundingBox_ymax(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_x_center ymax(::asurt_msgs::msg::BoundingBox::_ymax_type arg)
  {
    msg_.ymax = std::move(arg);
    return Init_BoundingBox_x_center(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_xmax
{
public:
  explicit Init_BoundingBox_xmax(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_ymax xmax(::asurt_msgs::msg::BoundingBox::_xmax_type arg)
  {
    msg_.xmax = std::move(arg);
    return Init_BoundingBox_ymax(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_ymin
{
public:
  explicit Init_BoundingBox_ymin(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_xmax ymin(::asurt_msgs::msg::BoundingBox::_ymin_type arg)
  {
    msg_.ymin = std::move(arg);
    return Init_BoundingBox_xmax(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_xmin
{
public:
  explicit Init_BoundingBox_xmin(::asurt_msgs::msg::BoundingBox & msg)
  : msg_(msg)
  {}
  Init_BoundingBox_ymin xmin(::asurt_msgs::msg::BoundingBox::_xmin_type arg)
  {
    msg_.xmin = std::move(arg);
    return Init_BoundingBox_ymin(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

class Init_BoundingBox_probability
{
public:
  Init_BoundingBox_probability()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BoundingBox_xmin probability(::asurt_msgs::msg::BoundingBox::_probability_type arg)
  {
    msg_.probability = std::move(arg);
    return Init_BoundingBox_xmin(msg_);
  }

private:
  ::asurt_msgs::msg::BoundingBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::BoundingBox>()
{
  return asurt_msgs::msg::builder::Init_BoundingBox_probability();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__BUILDER_HPP_
