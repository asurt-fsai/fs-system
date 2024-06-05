// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asurt_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
#define ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__asurt_msgs__msg__BoundingBox __attribute__((deprecated))
#else
# define DEPRECATED__asurt_msgs__msg__BoundingBox __declspec(deprecated)
#endif

namespace asurt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BoundingBox_
{
  using Type = BoundingBox_<ContainerAllocator>;

  explicit BoundingBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->probability = 0.0f;
      this->xmin = 0;
      this->ymin = 0;
      this->xmax = 0;
      this->ymax = 0;
      this->x_center = 0;
      this->y_center = 0;
      this->width = 0;
      this->height = 0;
      this->detection_id = 0;
      this->track_id = 0;
      this->type = 0;
    }
  }

  explicit BoundingBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->probability = 0.0f;
      this->xmin = 0;
      this->ymin = 0;
      this->xmax = 0;
      this->ymax = 0;
      this->x_center = 0;
      this->y_center = 0;
      this->width = 0;
      this->height = 0;
      this->detection_id = 0;
      this->track_id = 0;
      this->type = 0;
    }
  }

  // field types and members
  using _probability_type =
    float;
  _probability_type probability;
  using _xmin_type =
    uint16_t;
  _xmin_type xmin;
  using _ymin_type =
    uint16_t;
  _ymin_type ymin;
  using _xmax_type =
    uint16_t;
  _xmax_type xmax;
  using _ymax_type =
    uint16_t;
  _ymax_type ymax;
  using _x_center_type =
    uint16_t;
  _x_center_type x_center;
  using _y_center_type =
    uint16_t;
  _y_center_type y_center;
  using _width_type =
    uint16_t;
  _width_type width;
  using _height_type =
    uint16_t;
  _height_type height;
  using _detection_id_type =
    uint16_t;
  _detection_id_type detection_id;
  using _track_id_type =
    uint16_t;
  _track_id_type track_id;
  using _type_type =
    uint8_t;
  _type_type type;

  // setters for named parameter idiom
  Type & set__probability(
    const float & _arg)
  {
    this->probability = _arg;
    return *this;
  }
  Type & set__xmin(
    const uint16_t & _arg)
  {
    this->xmin = _arg;
    return *this;
  }
  Type & set__ymin(
    const uint16_t & _arg)
  {
    this->ymin = _arg;
    return *this;
  }
  Type & set__xmax(
    const uint16_t & _arg)
  {
    this->xmax = _arg;
    return *this;
  }
  Type & set__ymax(
    const uint16_t & _arg)
  {
    this->ymax = _arg;
    return *this;
  }
  Type & set__x_center(
    const uint16_t & _arg)
  {
    this->x_center = _arg;
    return *this;
  }
  Type & set__y_center(
    const uint16_t & _arg)
  {
    this->y_center = _arg;
    return *this;
  }
  Type & set__width(
    const uint16_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const uint16_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__detection_id(
    const uint16_t & _arg)
  {
    this->detection_id = _arg;
    return *this;
  }
  Type & set__track_id(
    const uint16_t & _arg)
  {
    this->track_id = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    asurt_msgs::msg::BoundingBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const asurt_msgs::msg::BoundingBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::BoundingBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::BoundingBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asurt_msgs__msg__BoundingBox
    std::shared_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asurt_msgs__msg__BoundingBox
    std::shared_ptr<asurt_msgs::msg::BoundingBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BoundingBox_ & other) const
  {
    if (this->probability != other.probability) {
      return false;
    }
    if (this->xmin != other.xmin) {
      return false;
    }
    if (this->ymin != other.ymin) {
      return false;
    }
    if (this->xmax != other.xmax) {
      return false;
    }
    if (this->ymax != other.ymax) {
      return false;
    }
    if (this->x_center != other.x_center) {
      return false;
    }
    if (this->y_center != other.y_center) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->detection_id != other.detection_id) {
      return false;
    }
    if (this->track_id != other.track_id) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const BoundingBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BoundingBox_

// alias to use template instance with default allocator
using BoundingBox =
  asurt_msgs::msg::BoundingBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_HPP_
