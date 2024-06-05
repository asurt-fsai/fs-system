// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/ConeImg.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__CONE_IMG__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__CONE_IMG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/cone_img__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeImg_track_id
{
public:
  explicit Init_ConeImg_track_id(::asurt_msgs::msg::ConeImg & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::ConeImg track_id(::asurt_msgs::msg::ConeImg::_track_id_type arg)
  {
    msg_.track_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImg msg_;
};

class Init_ConeImg_img
{
public:
  explicit Init_ConeImg_img(::asurt_msgs::msg::ConeImg & msg)
  : msg_(msg)
  {}
  Init_ConeImg_track_id img(::asurt_msgs::msg::ConeImg::_img_type arg)
  {
    msg_.img = std::move(arg);
    return Init_ConeImg_track_id(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImg msg_;
};

class Init_ConeImg_cols
{
public:
  explicit Init_ConeImg_cols(::asurt_msgs::msg::ConeImg & msg)
  : msg_(msg)
  {}
  Init_ConeImg_img cols(::asurt_msgs::msg::ConeImg::_cols_type arg)
  {
    msg_.cols = std::move(arg);
    return Init_ConeImg_img(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImg msg_;
};

class Init_ConeImg_rows
{
public:
  explicit Init_ConeImg_rows(::asurt_msgs::msg::ConeImg & msg)
  : msg_(msg)
  {}
  Init_ConeImg_cols rows(::asurt_msgs::msg::ConeImg::_rows_type arg)
  {
    msg_.rows = std::move(arg);
    return Init_ConeImg_cols(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImg msg_;
};

class Init_ConeImg_detection_id
{
public:
  Init_ConeImg_detection_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeImg_rows detection_id(::asurt_msgs::msg::ConeImg::_detection_id_type arg)
  {
    msg_.detection_id = std::move(arg);
    return Init_ConeImg_rows(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::ConeImg>()
{
  return asurt_msgs::msg::builder::Init_ConeImg_detection_id();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__CONE_IMG__BUILDER_HPP_
