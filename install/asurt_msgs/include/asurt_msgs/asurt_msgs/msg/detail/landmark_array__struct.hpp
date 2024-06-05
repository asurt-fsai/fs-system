// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asurt_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_HPP_
#define ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_HPP_

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
// Member 'landmarks'
#include "asurt_msgs/msg/detail/landmark__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__asurt_msgs__msg__LandmarkArray __attribute__((deprecated))
#else
# define DEPRECATED__asurt_msgs__msg__LandmarkArray __declspec(deprecated)
#endif

namespace asurt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LandmarkArray_
{
  using Type = LandmarkArray_<ContainerAllocator>;

  explicit LandmarkArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit LandmarkArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _landmarks_type =
    std::vector<asurt_msgs::msg::Landmark_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<asurt_msgs::msg::Landmark_<ContainerAllocator>>>;
  _landmarks_type landmarks;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__landmarks(
    const std::vector<asurt_msgs::msg::Landmark_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<asurt_msgs::msg::Landmark_<ContainerAllocator>>> & _arg)
  {
    this->landmarks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asurt_msgs::msg::LandmarkArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const asurt_msgs::msg::LandmarkArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::LandmarkArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::LandmarkArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asurt_msgs__msg__LandmarkArray
    std::shared_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asurt_msgs__msg__LandmarkArray
    std::shared_ptr<asurt_msgs::msg::LandmarkArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LandmarkArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->landmarks != other.landmarks) {
      return false;
    }
    return true;
  }
  bool operator!=(const LandmarkArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LandmarkArray_

// alias to use template instance with default allocator
using LandmarkArray =
  asurt_msgs::msg::LandmarkArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_HPP_
