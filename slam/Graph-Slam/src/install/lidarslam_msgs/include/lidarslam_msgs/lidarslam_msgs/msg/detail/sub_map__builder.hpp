// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidarslam_msgs:msg/SubMap.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__BUILDER_HPP_
#define LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "lidarslam_msgs/msg/detail/sub_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace lidarslam_msgs
{

namespace msg
{

namespace builder
{

class Init_SubMap_cloud
{
public:
  explicit Init_SubMap_cloud(::lidarslam_msgs::msg::SubMap & msg)
  : msg_(msg)
  {}
  ::lidarslam_msgs::msg::SubMap cloud(::lidarslam_msgs::msg::SubMap::_cloud_type arg)
  {
    msg_.cloud = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidarslam_msgs::msg::SubMap msg_;
};

class Init_SubMap_pose
{
public:
  explicit Init_SubMap_pose(::lidarslam_msgs::msg::SubMap & msg)
  : msg_(msg)
  {}
  Init_SubMap_cloud pose(::lidarslam_msgs::msg::SubMap::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_SubMap_cloud(msg_);
  }

private:
  ::lidarslam_msgs::msg::SubMap msg_;
};

class Init_SubMap_distance
{
public:
  explicit Init_SubMap_distance(::lidarslam_msgs::msg::SubMap & msg)
  : msg_(msg)
  {}
  Init_SubMap_pose distance(::lidarslam_msgs::msg::SubMap::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return Init_SubMap_pose(msg_);
  }

private:
  ::lidarslam_msgs::msg::SubMap msg_;
};

class Init_SubMap_header
{
public:
  Init_SubMap_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SubMap_distance header(::lidarslam_msgs::msg::SubMap::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SubMap_distance(msg_);
  }

private:
  ::lidarslam_msgs::msg::SubMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidarslam_msgs::msg::SubMap>()
{
  return lidarslam_msgs::msg::builder::Init_SubMap_header();
}

}  // namespace lidarslam_msgs

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__BUILDER_HPP_
