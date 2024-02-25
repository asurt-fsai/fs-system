// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidarslam_msgs:msg/SubMap.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__STRUCT_HPP_
#define LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidarslam_msgs__msg__SubMap __attribute__((deprecated))
#else
# define DEPRECATED__lidarslam_msgs__msg__SubMap __declspec(deprecated)
#endif

namespace lidarslam_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SubMap_
{
  using Type = SubMap_<ContainerAllocator>;

  explicit SubMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    cloud(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0;
    }
  }

  explicit SubMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init),
    cloud(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->distance = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _distance_type =
    double;
  _distance_type distance;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _cloud_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _cloud_type cloud;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__distance(
    const double & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__cloud(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->cloud = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidarslam_msgs::msg::SubMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidarslam_msgs::msg::SubMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidarslam_msgs::msg::SubMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidarslam_msgs::msg::SubMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidarslam_msgs__msg__SubMap
    std::shared_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidarslam_msgs__msg__SubMap
    std::shared_ptr<lidarslam_msgs::msg::SubMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SubMap_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->cloud != other.cloud) {
      return false;
    }
    return true;
  }
  bool operator!=(const SubMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SubMap_

// alias to use template instance with default allocator
using SubMap =
  lidarslam_msgs::msg::SubMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace lidarslam_msgs

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__STRUCT_HPP_
