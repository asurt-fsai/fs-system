// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asurt_msgs:msg/ConeImg.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__CONE_IMG__STRUCT_HPP_
#define ASURT_MSGS__MSG__DETAIL__CONE_IMG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'img'
#include "sensor_msgs/msg/detail/image__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__asurt_msgs__msg__ConeImg __attribute__((deprecated))
#else
# define DEPRECATED__asurt_msgs__msg__ConeImg __declspec(deprecated)
#endif

namespace asurt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConeImg_
{
  using Type = ConeImg_<ContainerAllocator>;

  explicit ConeImg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : img(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_id = 0;
      this->rows = 0;
      this->cols = 0;
      this->track_id = 0;
    }
  }

  explicit ConeImg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : img(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_id = 0;
      this->rows = 0;
      this->cols = 0;
      this->track_id = 0;
    }
  }

  // field types and members
  using _detection_id_type =
    uint16_t;
  _detection_id_type detection_id;
  using _rows_type =
    uint16_t;
  _rows_type rows;
  using _cols_type =
    uint16_t;
  _cols_type cols;
  using _img_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _img_type img;
  using _track_id_type =
    uint16_t;
  _track_id_type track_id;

  // setters for named parameter idiom
  Type & set__detection_id(
    const uint16_t & _arg)
  {
    this->detection_id = _arg;
    return *this;
  }
  Type & set__rows(
    const uint16_t & _arg)
  {
    this->rows = _arg;
    return *this;
  }
  Type & set__cols(
    const uint16_t & _arg)
  {
    this->cols = _arg;
    return *this;
  }
  Type & set__img(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->img = _arg;
    return *this;
  }
  Type & set__track_id(
    const uint16_t & _arg)
  {
    this->track_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asurt_msgs::msg::ConeImg_<ContainerAllocator> *;
  using ConstRawPtr =
    const asurt_msgs::msg::ConeImg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::ConeImg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::ConeImg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asurt_msgs__msg__ConeImg
    std::shared_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asurt_msgs__msg__ConeImg
    std::shared_ptr<asurt_msgs::msg::ConeImg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConeImg_ & other) const
  {
    if (this->detection_id != other.detection_id) {
      return false;
    }
    if (this->rows != other.rows) {
      return false;
    }
    if (this->cols != other.cols) {
      return false;
    }
    if (this->img != other.img) {
      return false;
    }
    if (this->track_id != other.track_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConeImg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConeImg_

// alias to use template instance with default allocator
using ConeImg =
  asurt_msgs::msg::ConeImg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__CONE_IMG__STRUCT_HPP_
