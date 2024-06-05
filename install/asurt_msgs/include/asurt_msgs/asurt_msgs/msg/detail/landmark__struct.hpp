// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asurt_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__LANDMARK__STRUCT_HPP_
#define ASURT_MSGS__MSG__DETAIL__LANDMARK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__asurt_msgs__msg__Landmark __attribute__((deprecated))
#else
# define DEPRECATED__asurt_msgs__msg__Landmark __declspec(deprecated)
#endif

namespace asurt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Landmark_
{
  using Type = Landmark_<ContainerAllocator>;

  explicit Landmark_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0ul;
      this->identifier = 0l;
      this->probability = 0.0;
    }
  }

  explicit Landmark_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0ul;
      this->identifier = 0l;
      this->probability = 0.0;
    }
  }

  // field types and members
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _type_type =
    uint32_t;
  _type_type type;
  using _identifier_type =
    int32_t;
  _identifier_type identifier;
  using _probability_type =
    double;
  _probability_type probability;

  // setters for named parameter idiom
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__type(
    const uint32_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__identifier(
    const int32_t & _arg)
  {
    this->identifier = _arg;
    return *this;
  }
  Type & set__probability(
    const double & _arg)
  {
    this->probability = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t BLUE_CONE =
    0u;
  static constexpr uint8_t YELLOW_CONE =
    1u;
  static constexpr uint8_t ORANGE_CONE =
    2u;
  static constexpr uint8_t LARGE_CONE =
    3u;
  static constexpr uint8_t CONE_TYPE_UNKNOWN =
    4u;

  // pointer types
  using RawPtr =
    asurt_msgs::msg::Landmark_<ContainerAllocator> *;
  using ConstRawPtr =
    const asurt_msgs::msg::Landmark_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::Landmark_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::Landmark_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asurt_msgs__msg__Landmark
    std::shared_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asurt_msgs__msg__Landmark
    std::shared_ptr<asurt_msgs::msg::Landmark_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Landmark_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->identifier != other.identifier) {
      return false;
    }
    if (this->probability != other.probability) {
      return false;
    }
    return true;
  }
  bool operator!=(const Landmark_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Landmark_

// alias to use template instance with default allocator
using Landmark =
  asurt_msgs::msg::Landmark_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::BLUE_CONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::YELLOW_CONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::ORANGE_CONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::LARGE_CONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Landmark_<ContainerAllocator>::CONE_TYPE_UNKNOWN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__LANDMARK__STRUCT_HPP_
