// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from asurt_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "asurt_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "asurt_msgs/msg/detail/landmark_array__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace asurt_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_asurt_msgs
cdr_serialize(
  const asurt_msgs::msg::LandmarkArray & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_asurt_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  asurt_msgs::msg::LandmarkArray & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_asurt_msgs
get_serialized_size(
  const asurt_msgs::msg::LandmarkArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_asurt_msgs
max_serialized_size_LandmarkArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace asurt_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_asurt_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, asurt_msgs, msg, LandmarkArray)();

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__LANDMARK_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
