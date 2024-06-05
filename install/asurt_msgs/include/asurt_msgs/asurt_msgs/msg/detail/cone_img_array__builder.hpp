// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from asurt_msgs:msg/ConeImgArray.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__CONE_IMG_ARRAY__BUILDER_HPP_
#define ASURT_MSGS__MSG__DETAIL__CONE_IMG_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "asurt_msgs/msg/detail/cone_img_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace asurt_msgs
{

namespace msg
{

namespace builder
{

class Init_ConeImgArray_imgs
{
public:
  explicit Init_ConeImgArray_imgs(::asurt_msgs::msg::ConeImgArray & msg)
  : msg_(msg)
  {}
  ::asurt_msgs::msg::ConeImgArray imgs(::asurt_msgs::msg::ConeImgArray::_imgs_type arg)
  {
    msg_.imgs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImgArray msg_;
};

class Init_ConeImgArray_object_count
{
public:
  explicit Init_ConeImgArray_object_count(::asurt_msgs::msg::ConeImgArray & msg)
  : msg_(msg)
  {}
  Init_ConeImgArray_imgs object_count(::asurt_msgs::msg::ConeImgArray::_object_count_type arg)
  {
    msg_.object_count = std::move(arg);
    return Init_ConeImgArray_imgs(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImgArray msg_;
};

class Init_ConeImgArray_view_id
{
public:
  Init_ConeImgArray_view_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConeImgArray_object_count view_id(::asurt_msgs::msg::ConeImgArray::_view_id_type arg)
  {
    msg_.view_id = std::move(arg);
    return Init_ConeImgArray_object_count(msg_);
  }

private:
  ::asurt_msgs::msg::ConeImgArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::asurt_msgs::msg::ConeImgArray>()
{
  return asurt_msgs::msg::builder::Init_ConeImgArray_view_id();
}

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__CONE_IMG_ARRAY__BUILDER_HPP_
