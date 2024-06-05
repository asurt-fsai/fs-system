// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from asurt_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "asurt_msgs/msg/detail/landmark_array__struct.hpp"
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

void LandmarkArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) asurt_msgs::msg::LandmarkArray(_init);
}

void LandmarkArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<asurt_msgs::msg::LandmarkArray *>(message_memory);
  typed_message->~LandmarkArray();
}

size_t size_function__LandmarkArray__landmarks(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<asurt_msgs::msg::Landmark> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LandmarkArray__landmarks(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<asurt_msgs::msg::Landmark> *>(untyped_member);
  return &member[index];
}

void * get_function__LandmarkArray__landmarks(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<asurt_msgs::msg::Landmark> *>(untyped_member);
  return &member[index];
}

void fetch_function__LandmarkArray__landmarks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const asurt_msgs::msg::Landmark *>(
    get_const_function__LandmarkArray__landmarks(untyped_member, index));
  auto & value = *reinterpret_cast<asurt_msgs::msg::Landmark *>(untyped_value);
  value = item;
}

void assign_function__LandmarkArray__landmarks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<asurt_msgs::msg::Landmark *>(
    get_function__LandmarkArray__landmarks(untyped_member, index));
  const auto & value = *reinterpret_cast<const asurt_msgs::msg::Landmark *>(untyped_value);
  item = value;
}

void resize_function__LandmarkArray__landmarks(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<asurt_msgs::msg::Landmark> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LandmarkArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs::msg::LandmarkArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "landmarks",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<asurt_msgs::msg::Landmark>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs::msg::LandmarkArray, landmarks),  // bytes offset in struct
    nullptr,  // default value
    size_function__LandmarkArray__landmarks,  // size() function pointer
    get_const_function__LandmarkArray__landmarks,  // get_const(index) function pointer
    get_function__LandmarkArray__landmarks,  // get(index) function pointer
    fetch_function__LandmarkArray__landmarks,  // fetch(index, &value) function pointer
    assign_function__LandmarkArray__landmarks,  // assign(index, value) function pointer
    resize_function__LandmarkArray__landmarks  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LandmarkArray_message_members = {
  "asurt_msgs::msg",  // message namespace
  "LandmarkArray",  // message name
  2,  // number of fields
  sizeof(asurt_msgs::msg::LandmarkArray),
  LandmarkArray_message_member_array,  // message members
  LandmarkArray_init_function,  // function to initialize message memory (memory has to be allocated)
  LandmarkArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LandmarkArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LandmarkArray_message_members,
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
get_message_type_support_handle<asurt_msgs::msg::LandmarkArray>()
{
  return &::asurt_msgs::msg::rosidl_typesupport_introspection_cpp::LandmarkArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, asurt_msgs, msg, LandmarkArray)() {
  return &::asurt_msgs::msg::rosidl_typesupport_introspection_cpp::LandmarkArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
