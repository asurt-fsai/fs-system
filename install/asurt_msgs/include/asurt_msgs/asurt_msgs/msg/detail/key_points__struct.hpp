// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asurt_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_HPP_
#define ASURT_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__asurt_msgs__msg__KeyPoints __attribute__((deprecated))
#else
# define DEPRECATED__asurt_msgs__msg__KeyPoints __declspec(deprecated)
#endif

namespace asurt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KeyPoints_
{
  using Type = KeyPoints_<ContainerAllocator>;

  explicit KeyPoints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->view_id = 0ul;
      this->object_count = 0;
    }
  }

  explicit KeyPoints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->view_id = 0ul;
      this->object_count = 0;
    }
  }

  // field types and members
  using _view_id_type =
    uint32_t;
  _view_id_type view_id;
  using _object_count_type =
    uint16_t;
  _object_count_type object_count;
  using _track_ids_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _track_ids_type track_ids;
  using _classes_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _classes_type classes;
  using _keypoints_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _keypoints_type keypoints;

  // setters for named parameter idiom
  Type & set__view_id(
    const uint32_t & _arg)
  {
    this->view_id = _arg;
    return *this;
  }
  Type & set__object_count(
    const uint16_t & _arg)
  {
    this->object_count = _arg;
    return *this;
  }
  Type & set__track_ids(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->track_ids = _arg;
    return *this;
  }
  Type & set__classes(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->classes = _arg;
    return *this;
  }
  Type & set__keypoints(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->keypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asurt_msgs::msg::KeyPoints_<ContainerAllocator> *;
  using ConstRawPtr =
    const asurt_msgs::msg::KeyPoints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::KeyPoints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::KeyPoints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asurt_msgs__msg__KeyPoints
    std::shared_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asurt_msgs__msg__KeyPoints
    std::shared_ptr<asurt_msgs::msg::KeyPoints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyPoints_ & other) const
  {
    if (this->view_id != other.view_id) {
      return false;
    }
    if (this->object_count != other.object_count) {
      return false;
    }
    if (this->track_ids != other.track_ids) {
      return false;
    }
    if (this->classes != other.classes) {
      return false;
    }
    if (this->keypoints != other.keypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyPoints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyPoints_

// alias to use template instance with default allocator
using KeyPoints =
  asurt_msgs::msg::KeyPoints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_HPP_
