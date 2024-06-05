// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from asurt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
#define ASURT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_

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
# define DEPRECATED__asurt_msgs__msg__NodeStatus __attribute__((deprecated))
#else
# define DEPRECATED__asurt_msgs__msg__NodeStatus __declspec(deprecated)
#endif

namespace asurt_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NodeStatus_
{
  using Type = NodeStatus_<ContainerAllocator>;

  explicit NodeStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->message = "";
    }
  }

  explicit NodeStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
      this->message = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _status_type =
    uint8_t;
  _status_type status;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__status(
    const uint8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t STARTING =
    0u;
  static constexpr uint8_t READY =
    1u;
  static constexpr uint8_t RUNNING =
    2u;
  // guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
  static constexpr uint8_t ERROR =
    3u;
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
  static constexpr uint8_t SHUTDOWN =
    4u;
  static constexpr uint8_t UNRESPONSIVE =
    5u;

  // pointer types
  using RawPtr =
    asurt_msgs::msg::NodeStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const asurt_msgs::msg::NodeStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::NodeStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      asurt_msgs::msg::NodeStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__asurt_msgs__msg__NodeStatus
    std::shared_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__asurt_msgs__msg__NodeStatus
    std::shared_ptr<asurt_msgs::msg::NodeStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NodeStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const NodeStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NodeStatus_

// alias to use template instance with default allocator
using NodeStatus =
  asurt_msgs::msg::NodeStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NodeStatus_<ContainerAllocator>::STARTING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NodeStatus_<ContainerAllocator>::READY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NodeStatus_<ContainerAllocator>::RUNNING;
#endif  // __cplusplus < 201703L
// guard against 'ERROR' being predefined by MSVC by temporarily undefining it
#if defined(_WIN32)
#  if defined(ERROR)
#    pragma push_macro("ERROR")
#    undef ERROR
#  endif
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NodeStatus_<ContainerAllocator>::ERROR;
#endif  // __cplusplus < 201703L
#if defined(_WIN32)
#  pragma warning(suppress : 4602)
#  pragma pop_macro("ERROR")
#endif
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NodeStatus_<ContainerAllocator>::SHUTDOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t NodeStatus_<ContainerAllocator>::UNRESPONSIVE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace asurt_msgs

#endif  // ASURT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_HPP_
