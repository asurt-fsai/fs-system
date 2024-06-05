// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asurt_msgs:msg/ConeImg.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asurt_msgs/msg/detail/cone_img__rosidl_typesupport_introspection_c.h"
#include "asurt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asurt_msgs/msg/detail/cone_img__functions.h"
#include "asurt_msgs/msg/detail/cone_img__struct.h"


// Include directives for member types
// Member `img`
#include "sensor_msgs/msg/image.h"
// Member `img`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asurt_msgs__msg__ConeImg__init(message_memory);
}

void asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_fini_function(void * message_memory)
{
  asurt_msgs__msg__ConeImg__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_member_array[5] = {
  {
    "detection_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__ConeImg, detection_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rows",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__ConeImg, rows),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cols",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__ConeImg, cols),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "img",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__ConeImg, img),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "track_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__ConeImg, track_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_members = {
  "asurt_msgs__msg",  // message namespace
  "ConeImg",  // message name
  5,  // number of fields
  sizeof(asurt_msgs__msg__ConeImg),
  asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_member_array,  // message members
  asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_init_function,  // function to initialize message memory (memory has to be allocated)
  asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_type_support_handle = {
  0,
  &asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asurt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, ConeImg)() {
  asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  if (!asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_type_support_handle.typesupport_identifier) {
    asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asurt_msgs__msg__ConeImg__rosidl_typesupport_introspection_c__ConeImg_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
