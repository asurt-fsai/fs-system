// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asurt_msgs:msg/BoundingBoxes.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asurt_msgs/msg/detail/bounding_boxes__rosidl_typesupport_introspection_c.h"
#include "asurt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asurt_msgs/msg/detail/bounding_boxes__functions.h"
#include "asurt_msgs/msg/detail/bounding_boxes__struct.h"


// Include directives for member types
// Member `bounding_boxes`
#include "asurt_msgs/msg/bounding_box.h"
// Member `bounding_boxes`
#include "asurt_msgs/msg/detail/bounding_box__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asurt_msgs__msg__BoundingBoxes__init(message_memory);
}

void asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_fini_function(void * message_memory)
{
  asurt_msgs__msg__BoundingBoxes__fini(message_memory);
}

size_t asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__size_function__BoundingBoxes__bounding_boxes(
  const void * untyped_member)
{
  const asurt_msgs__msg__BoundingBox__Sequence * member =
    (const asurt_msgs__msg__BoundingBox__Sequence *)(untyped_member);
  return member->size;
}

const void * asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__bounding_boxes(
  const void * untyped_member, size_t index)
{
  const asurt_msgs__msg__BoundingBox__Sequence * member =
    (const asurt_msgs__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void * asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__bounding_boxes(
  void * untyped_member, size_t index)
{
  asurt_msgs__msg__BoundingBox__Sequence * member =
    (asurt_msgs__msg__BoundingBox__Sequence *)(untyped_member);
  return &member->data[index];
}

void asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__fetch_function__BoundingBoxes__bounding_boxes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const asurt_msgs__msg__BoundingBox * item =
    ((const asurt_msgs__msg__BoundingBox *)
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__bounding_boxes(untyped_member, index));
  asurt_msgs__msg__BoundingBox * value =
    (asurt_msgs__msg__BoundingBox *)(untyped_value);
  *value = *item;
}

void asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__assign_function__BoundingBoxes__bounding_boxes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  asurt_msgs__msg__BoundingBox * item =
    ((asurt_msgs__msg__BoundingBox *)
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__bounding_boxes(untyped_member, index));
  const asurt_msgs__msg__BoundingBox * value =
    (const asurt_msgs__msg__BoundingBox *)(untyped_value);
  *item = *value;
}

bool asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__resize_function__BoundingBoxes__bounding_boxes(
  void * untyped_member, size_t size)
{
  asurt_msgs__msg__BoundingBox__Sequence * member =
    (asurt_msgs__msg__BoundingBox__Sequence *)(untyped_member);
  asurt_msgs__msg__BoundingBox__Sequence__fini(member);
  return asurt_msgs__msg__BoundingBox__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array[3] = {
  {
    "view_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__BoundingBoxes, view_id),  // bytes offset in struct
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
    offsetof(asurt_msgs__msg__BoundingBoxes, object_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounding_boxes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__BoundingBoxes, bounding_boxes),  // bytes offset in struct
    NULL,  // default value
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__size_function__BoundingBoxes__bounding_boxes,  // size() function pointer
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_const_function__BoundingBoxes__bounding_boxes,  // get_const(index) function pointer
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__get_function__BoundingBoxes__bounding_boxes,  // get(index) function pointer
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__fetch_function__BoundingBoxes__bounding_boxes,  // fetch(index, &value) function pointer
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__assign_function__BoundingBoxes__bounding_boxes,  // assign(index, value) function pointer
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__resize_function__BoundingBoxes__bounding_boxes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_members = {
  "asurt_msgs__msg",  // message namespace
  "BoundingBoxes",  // message name
  3,  // number of fields
  sizeof(asurt_msgs__msg__BoundingBoxes),
  asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array,  // message members
  asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_init_function,  // function to initialize message memory (memory has to be allocated)
  asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle = {
  0,
  &asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asurt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, BoundingBoxes)() {
  asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, BoundingBox)();
  if (!asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle.typesupport_identifier) {
    asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asurt_msgs__msg__BoundingBoxes__rosidl_typesupport_introspection_c__BoundingBoxes_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
