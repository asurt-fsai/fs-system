// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asurt_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asurt_msgs/msg/detail/key_points__rosidl_typesupport_introspection_c.h"
#include "asurt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asurt_msgs/msg/detail/key_points__functions.h"
#include "asurt_msgs/msg/detail/key_points__struct.h"


// Include directives for member types
// Member `track_ids`
// Member `classes`
// Member `keypoints`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asurt_msgs__msg__KeyPoints__init(message_memory);
}

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_fini_function(void * message_memory)
{
  asurt_msgs__msg__KeyPoints__fini(message_memory);
}

size_t asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__size_function__KeyPoints__track_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__track_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__track_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__fetch_function__KeyPoints__track_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__track_ids(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__assign_function__KeyPoints__track_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__track_ids(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__resize_function__KeyPoints__track_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__size_function__KeyPoints__classes(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__classes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__classes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__fetch_function__KeyPoints__classes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__classes(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__assign_function__KeyPoints__classes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__classes(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__resize_function__KeyPoints__classes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__size_function__KeyPoints__keypoints(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__keypoints(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__keypoints(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__fetch_function__KeyPoints__keypoints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__keypoints(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__assign_function__KeyPoints__keypoints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__keypoints(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__resize_function__KeyPoints__keypoints(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_member_array[5] = {
  {
    "view_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__KeyPoints, view_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__KeyPoints, object_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "track_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__KeyPoints, track_ids),  // bytes offset in struct
    NULL,  // default value
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__size_function__KeyPoints__track_ids,  // size() function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__track_ids,  // get_const(index) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__track_ids,  // get(index) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__fetch_function__KeyPoints__track_ids,  // fetch(index, &value) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__assign_function__KeyPoints__track_ids,  // assign(index, value) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__resize_function__KeyPoints__track_ids  // resize(index) function pointer
  },
  {
    "classes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__KeyPoints, classes),  // bytes offset in struct
    NULL,  // default value
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__size_function__KeyPoints__classes,  // size() function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__classes,  // get_const(index) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__classes,  // get(index) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__fetch_function__KeyPoints__classes,  // fetch(index, &value) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__assign_function__KeyPoints__classes,  // assign(index, value) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__resize_function__KeyPoints__classes  // resize(index) function pointer
  },
  {
    "keypoints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__KeyPoints, keypoints),  // bytes offset in struct
    NULL,  // default value
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__size_function__KeyPoints__keypoints,  // size() function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_const_function__KeyPoints__keypoints,  // get_const(index) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__get_function__KeyPoints__keypoints,  // get(index) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__fetch_function__KeyPoints__keypoints,  // fetch(index, &value) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__assign_function__KeyPoints__keypoints,  // assign(index, value) function pointer
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__resize_function__KeyPoints__keypoints  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_members = {
  "asurt_msgs__msg",  // message namespace
  "KeyPoints",  // message name
  5,  // number of fields
  sizeof(asurt_msgs__msg__KeyPoints),
  asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_member_array,  // message members
  asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_init_function,  // function to initialize message memory (memory has to be allocated)
  asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle = {
  0,
  &asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asurt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, KeyPoints)() {
  if (!asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle.typesupport_identifier) {
    asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asurt_msgs__msg__KeyPoints__rosidl_typesupport_introspection_c__KeyPoints_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
