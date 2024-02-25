// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidarslam_msgs:msg/MapArray.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__BUILDER_HPP_
#define LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidarslam_msgs/msg/detail/map_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidarslam_msgs
{

namespace msg
{

namespace builder
{

class Init_MapArray_cloud_coordinate
{
public:
  explicit Init_MapArray_cloud_coordinate(::lidarslam_msgs::msg::MapArray & msg)
  : msg_(msg)
  {}
  ::lidarslam_msgs::msg::MapArray cloud_coordinate(::lidarslam_msgs::msg::MapArray::_cloud_coordinate_type arg)
  {
    msg_.cloud_coordinate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidarslam_msgs::msg::MapArray msg_;
};

class Init_MapArray_submaps
{
public:
  explicit Init_MapArray_submaps(::lidarslam_msgs::msg::MapArray & msg)
  : msg_(msg)
  {}
  Init_MapArray_cloud_coordinate submaps(::lidarslam_msgs::msg::MapArray::_submaps_type arg)
  {
    msg_.submaps = std::move(arg);
    return Init_MapArray_cloud_coordinate(msg_);
  }

private:
  ::lidarslam_msgs::msg::MapArray msg_;
};

class Init_MapArray_header
{
public:
  Init_MapArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MapArray_submaps header(::lidarslam_msgs::msg::MapArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MapArray_submaps(msg_);
  }

private:
  ::lidarslam_msgs::msg::MapArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidarslam_msgs::msg::MapArray>()
{
  return lidarslam_msgs::msg::builder::Init_MapArray_header();
}

}  // namespace lidarslam_msgs

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__BUILDER_HPP_
