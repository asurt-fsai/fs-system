// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asurt_msgs:msg/Roadstate.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__ROADSTATE__STRUCT_HPP_
#define ASURT_MSGS__MSG__DETAIL__ROADSTATE__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__asurt_msgs__msg__Roadstate __attribute__((deprecated))
#else
# define DEPRECATED__asurt_msgs__msg__Roadstate __declspec(deprecated)
#endif

namespace asurt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Roadstate_
{
  using Type = Roadstate_<ContainerAllocator>;

  explicit Roadstate_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->laps = 0l;
      this->distance = 0.0f;
    }
  }

  explicit Roadstate_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->laps = 0l;
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _laps_type =
    int32_t;
  _laps_type laps;
  using _distance_type =
    float;
  _distance_type distance;
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;

  // setters for named parameter idiom
  Type & set__laps(
    const int32_t & _arg)
  {
    this->laps = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asurt_msgs::msg::Roadstate_<ContainerAllocator> *;
  using ConstRawPtr =
    const asurt_msgs::msg::Roadstate_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::Roadstate_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::Roadstate_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asurt_msgs__msg__Roadstate
    std::shared_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asurt_msgs__msg__Roadstate
    std::shared_ptr<asurt_msgs::msg::Roadstate_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Roadstate_ & other) const
  {
    if (this->laps != other.laps) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    if (this->header != other.header) {
      return false;
    }
    return true;
  }
  bool operator!=(const Roadstate_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Roadstate_

// alias to use template instance with default allocator
using Roadstate =
  asurt_msgs::msg::Roadstate_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__ROADSTATE__STRUCT_HPP_
