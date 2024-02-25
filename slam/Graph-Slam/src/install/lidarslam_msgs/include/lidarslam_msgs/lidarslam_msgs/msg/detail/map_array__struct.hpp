// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidarslam_msgs:msg/MapArray.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__STRUCT_HPP_
#define LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__STRUCT_HPP_

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
// Member 'submaps'
#include "lidarslam_msgs/msg/detail/sub_map__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidarslam_msgs__msg__MapArray __attribute__((deprecated))
#else
# define DEPRECATED__lidarslam_msgs__msg__MapArray __declspec(deprecated)
#endif

namespace lidarslam_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MapArray_
{
  using Type = MapArray_<ContainerAllocator>;

  explicit MapArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cloud_coordinate = 0;
    }
  }

  explicit MapArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cloud_coordinate = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _submaps_type =
    std::vector<lidarslam_msgs::msg::SubMap_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<lidarslam_msgs::msg::SubMap_<ContainerAllocator>>>;
  _submaps_type submaps;
  using _cloud_coordinate_type =
    int8_t;
  _cloud_coordinate_type cloud_coordinate;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__submaps(
    const std::vector<lidarslam_msgs::msg::SubMap_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<lidarslam_msgs::msg::SubMap_<ContainerAllocator>>> & _arg)
  {
    this->submaps = _arg;
    return *this;
  }
  Type & set__cloud_coordinate(
    const int8_t & _arg)
  {
    this->cloud_coordinate = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t LOCAL =
    0;
  static constexpr int8_t GLOBAL =
    1;

  // pointer types
  using RawPtr =
    lidarslam_msgs::msg::MapArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidarslam_msgs::msg::MapArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidarslam_msgs::msg::MapArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidarslam_msgs::msg::MapArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidarslam_msgs__msg__MapArray
    std::shared_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidarslam_msgs__msg__MapArray
    std::shared_ptr<lidarslam_msgs::msg::MapArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MapArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->submaps != other.submaps) {
      return false;
    }
    if (this->cloud_coordinate != other.cloud_coordinate) {
      return false;
    }
    return true;
  }
  bool operator!=(const MapArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MapArray_

// alias to use template instance with default allocator
using MapArray =
  lidarslam_msgs::msg::MapArray_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t MapArray_<ContainerAllocator>::LOCAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t MapArray_<ContainerAllocator>::GLOBAL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace lidarslam_msgs

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__STRUCT_HPP_
