// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from asurt_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "asurt_msgs/msg/detail/key_points__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace asurt_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void KeyPoints_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) asurt_msgs::msg::KeyPoints(_init);
}

void KeyPoints_fini_function(void * message_memory)
{
  auto typed_message = static_cast<asurt_msgs::msg::KeyPoints *>(message_memory);
  typed_message->~KeyPoints();
}

size_t size_function__KeyPoints__track_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__KeyPoints__track_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__KeyPoints__track_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__KeyPoints__track_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__KeyPoints__track_ids(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__KeyPoints__track_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__KeyPoints__track_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__KeyPoints__track_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__KeyPoints__classes(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__KeyPoints__classes(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__KeyPoints__classes(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__KeyPoints__classes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__KeyPoints__classes(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__KeyPoints__classes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__KeyPoints__classes(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__KeyPoints__classes(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__KeyPoints__keypoints(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__KeyPoints__keypoints(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void * get_function__KeyPoints__keypoints(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__KeyPoints__keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint16_t *>(
    get_const_function__KeyPoints__keypoints(untyped_member, index));
  auto & value = *reinterpret_cast<uint16_t *>(untyped_value);
  value = item;
}

void assign_function__KeyPoints__keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint16_t *>(
    get_function__KeyPoints__keypoints(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint16_t *>(untyped_value);
  item = value;
}

void resize_function__KeyPoints__keypoints(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint16_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember KeyPoints_message_member_array[5] = {
  {
    "view_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs::msg::KeyPoints, view_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "object_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs::msg::KeyPoints, object_count),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "track_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs::msg::KeyPoints, track_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__KeyPoints__track_ids,  // size() function pointer
    get_const_function__KeyPoints__track_ids,  // get_const(index) function pointer
    get_function__KeyPoints__track_ids,  // get(index) function pointer
    fetch_function__KeyPoints__track_ids,  // fetch(index, &value) function pointer
    assign_function__KeyPoints__track_ids,  // assign(index, value) function pointer
    resize_function__KeyPoints__track_ids  // resize(index) function pointer
  },
  {
    "classes",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs::msg::KeyPoints, classes),  // bytes offset in struct
    nullptr,  // default value
    size_function__KeyPoints__classes,  // size() function pointer
    get_const_function__KeyPoints__classes,  // get_const(index) function pointer
    get_function__KeyPoints__classes,  // get(index) function pointer
    fetch_function__KeyPoints__classes,  // fetch(index, &value) function pointer
    assign_function__KeyPoints__classes,  // assign(index, value) function pointer
    resize_function__KeyPoints__classes  // resize(index) function pointer
  },
  {
    "keypoints",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs::msg::KeyPoints, keypoints),  // bytes offset in struct
    nullptr,  // default value
    size_function__KeyPoints__keypoints,  // size() function pointer
    get_const_function__KeyPoints__keypoints,  // get_const(index) function pointer
    get_function__KeyPoints__keypoints,  // get(index) function pointer
    fetch_function__KeyPoints__keypoints,  // fetch(index, &value) function pointer
    assign_function__KeyPoints__keypoints,  // assign(index, value) function pointer
    resize_function__KeyPoints__keypoints  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers KeyPoints_message_members = {
  "asurt_msgs::msg",  // message namespace
  "KeyPoints",  // message name
  5,  // number of fields
  sizeof(asurt_msgs::msg::KeyPoints),
  KeyPoints_message_member_array,  // message members
  KeyPoints_init_function,  // function to initialize message memory (memory has to be allocated)
  KeyPoints_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t KeyPoints_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &KeyPoints_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace asurt_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<asurt_msgs::msg::KeyPoints>()
{
  return &::asurt_msgs::msg::rosidl_typesupport_introspection_cpp::KeyPoints_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asurt_msgs, msg, KeyPoints)() {
  return &::asurt_msgs::msg::rosidl_typesupport_introspection_cpp::KeyPoints_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
