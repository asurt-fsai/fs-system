// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asurt_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asurt_msgs/msg/detail/landmark__rosidl_typesupport_introspection_c.h"
#include "asurt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asurt_msgs/msg/detail/landmark__functions.h"
#include "asurt_msgs/msg/detail/landmark__struct.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asurt_msgs__msg__Landmark__init(message_memory);
}

void asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_fini_function(void * message_memory)
{
  asurt_msgs__msg__Landmark__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_member_array[4] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__Landmark, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__Landmark, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "identifier",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__Landmark, identifier),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "probability",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__Landmark, probability),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_members = {
  "asurt_msgs__msg",  // message namespace
  "Landmark",  // message name
  4,  // number of fields
  sizeof(asurt_msgs__msg__Landmark),
  asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_member_array,  // message members
  asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_init_function,  // function to initialize message memory (memory has to be allocated)
  asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle = {
  0,
  &asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asurt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, Landmark)() {
  asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle.typesupport_identifier) {
    asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asurt_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
