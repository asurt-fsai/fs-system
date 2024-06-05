// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asurt_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asurt_msgs/msg/detail/landmark_array__rosidl_typesupport_introspection_c.h"
#include "asurt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asurt_msgs/msg/detail/landmark_array__functions.h"
#include "asurt_msgs/msg/detail/landmark_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `landmarks`
#include "asurt_msgs/msg/landmark.h"
// Member `landmarks`
#include "asurt_msgs/msg/detail/landmark__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asurt_msgs__msg__LandmarkArray__init(message_memory);
}

void asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_fini_function(void * message_memory)
{
  asurt_msgs__msg__LandmarkArray__fini(message_memory);
}

size_t asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__size_function__LandmarkArray__landmarks(
  const void * untyped_member)
{
  const asurt_msgs__msg__Landmark__Sequence * member =
    (const asurt_msgs__msg__Landmark__Sequence *)(untyped_member);
  return member->size;
}

const void * asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__get_const_function__LandmarkArray__landmarks(
  const void * untyped_member, size_t index)
{
  const asurt_msgs__msg__Landmark__Sequence * member =
    (const asurt_msgs__msg__Landmark__Sequence *)(untyped_member);
  return &member->data[index];
}

void * asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__get_function__LandmarkArray__landmarks(
  void * untyped_member, size_t index)
{
  asurt_msgs__msg__Landmark__Sequence * member =
    (asurt_msgs__msg__Landmark__Sequence *)(untyped_member);
  return &member->data[index];
}

void asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__fetch_function__LandmarkArray__landmarks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const asurt_msgs__msg__Landmark * item =
    ((const asurt_msgs__msg__Landmark *)
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__get_const_function__LandmarkArray__landmarks(untyped_member, index));
  asurt_msgs__msg__Landmark * value =
    (asurt_msgs__msg__Landmark *)(untyped_value);
  *value = *item;
}

void asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__assign_function__LandmarkArray__landmarks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  asurt_msgs__msg__Landmark * item =
    ((asurt_msgs__msg__Landmark *)
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__get_function__LandmarkArray__landmarks(untyped_member, index));
  const asurt_msgs__msg__Landmark * value =
    (const asurt_msgs__msg__Landmark *)(untyped_value);
  *item = *value;
}

bool asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__resize_function__LandmarkArray__landmarks(
  void * untyped_member, size_t size)
{
  asurt_msgs__msg__Landmark__Sequence * member =
    (asurt_msgs__msg__Landmark__Sequence *)(untyped_member);
  asurt_msgs__msg__Landmark__Sequence__fini(member);
  return asurt_msgs__msg__Landmark__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__LandmarkArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "landmarks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__LandmarkArray, landmarks),  // bytes offset in struct
    NULL,  // default value
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__size_function__LandmarkArray__landmarks,  // size() function pointer
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__get_const_function__LandmarkArray__landmarks,  // get_const(index) function pointer
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__get_function__LandmarkArray__landmarks,  // get(index) function pointer
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__fetch_function__LandmarkArray__landmarks,  // fetch(index, &value) function pointer
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__assign_function__LandmarkArray__landmarks,  // assign(index, value) function pointer
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__resize_function__LandmarkArray__landmarks  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_members = {
  "asurt_msgs__msg",  // message namespace
  "LandmarkArray",  // message name
  2,  // number of fields
  sizeof(asurt_msgs__msg__LandmarkArray),
  asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_member_array,  // message members
  asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_init_function,  // function to initialize message memory (memory has to be allocated)
  asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_type_support_handle = {
  0,
  &asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asurt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, LandmarkArray)() {
  asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, Landmark)();
  if (!asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_type_support_handle.typesupport_identifier) {
    asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asurt_msgs__msg__LandmarkArray__rosidl_typesupport_introspection_c__LandmarkArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
