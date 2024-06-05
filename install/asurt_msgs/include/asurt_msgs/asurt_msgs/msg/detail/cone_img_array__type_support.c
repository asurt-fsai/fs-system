// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from asurt_msgs:msg/ConeImgArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "asurt_msgs/msg/detail/cone_img_array__rosidl_typesupport_introspection_c.h"
#include "asurt_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "asurt_msgs/msg/detail/cone_img_array__functions.h"
#include "asurt_msgs/msg/detail/cone_img_array__struct.h"


// Include directives for member types
// Member `imgs`
#include "asurt_msgs/msg/cone_img.h"
// Member `imgs`
#include "asurt_msgs/msg/detail/cone_img__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  asurt_msgs__msg__ConeImgArray__init(message_memory);
}

void asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_fini_function(void * message_memory)
{
  asurt_msgs__msg__ConeImgArray__fini(message_memory);
}

size_t asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__size_function__ConeImgArray__imgs(
  const void * untyped_member)
{
  const asurt_msgs__msg__ConeImg__Sequence * member =
    (const asurt_msgs__msg__ConeImg__Sequence *)(untyped_member);
  return member->size;
}

const void * asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__get_const_function__ConeImgArray__imgs(
  const void * untyped_member, size_t index)
{
  const asurt_msgs__msg__ConeImg__Sequence * member =
    (const asurt_msgs__msg__ConeImg__Sequence *)(untyped_member);
  return &member->data[index];
}

void * asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__get_function__ConeImgArray__imgs(
  void * untyped_member, size_t index)
{
  asurt_msgs__msg__ConeImg__Sequence * member =
    (asurt_msgs__msg__ConeImg__Sequence *)(untyped_member);
  return &member->data[index];
}

void asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__fetch_function__ConeImgArray__imgs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const asurt_msgs__msg__ConeImg * item =
    ((const asurt_msgs__msg__ConeImg *)
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__get_const_function__ConeImgArray__imgs(untyped_member, index));
  asurt_msgs__msg__ConeImg * value =
    (asurt_msgs__msg__ConeImg *)(untyped_value);
  *value = *item;
}

void asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__assign_function__ConeImgArray__imgs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  asurt_msgs__msg__ConeImg * item =
    ((asurt_msgs__msg__ConeImg *)
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__get_function__ConeImgArray__imgs(untyped_member, index));
  const asurt_msgs__msg__ConeImg * value =
    (const asurt_msgs__msg__ConeImg *)(untyped_value);
  *item = *value;
}

bool asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__resize_function__ConeImgArray__imgs(
  void * untyped_member, size_t size)
{
  asurt_msgs__msg__ConeImg__Sequence * member =
    (asurt_msgs__msg__ConeImg__Sequence *)(untyped_member);
  asurt_msgs__msg__ConeImg__Sequence__fini(member);
  return asurt_msgs__msg__ConeImg__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_member_array[3] = {
  {
    "view_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__ConeImgArray, view_id),  // bytes offset in struct
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
    offsetof(asurt_msgs__msg__ConeImgArray, object_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imgs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(asurt_msgs__msg__ConeImgArray, imgs),  // bytes offset in struct
    NULL,  // default value
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__size_function__ConeImgArray__imgs,  // size() function pointer
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__get_const_function__ConeImgArray__imgs,  // get_const(index) function pointer
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__get_function__ConeImgArray__imgs,  // get(index) function pointer
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__fetch_function__ConeImgArray__imgs,  // fetch(index, &value) function pointer
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__assign_function__ConeImgArray__imgs,  // assign(index, value) function pointer
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__resize_function__ConeImgArray__imgs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_members = {
  "asurt_msgs__msg",  // message namespace
  "ConeImgArray",  // message name
  3,  // number of fields
  sizeof(asurt_msgs__msg__ConeImgArray),
  asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_member_array,  // message members
  asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_init_function,  // function to initialize message memory (memory has to be allocated)
  asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_type_support_handle = {
  0,
  &asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_asurt_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, ConeImgArray)() {
  asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, asurt_msgs, msg, ConeImg)();
  if (!asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_type_support_handle.typesupport_identifier) {
    asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &asurt_msgs__msg__ConeImgArray__rosidl_typesupport_introspection_c__ConeImgArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
