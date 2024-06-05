// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asurt_msgs:msg/ConeImgArray.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__CONE_IMG_ARRAY__STRUCT_H_
#define ASURT_MSGS__MSG__DETAIL__CONE_IMG_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'imgs'
#include "asurt_msgs/msg/detail/cone_img__struct.h"

/// Struct defined in msg/ConeImgArray in the package asurt_msgs.
typedef struct asurt_msgs__msg__ConeImgArray
{
  uint32_t view_id;
  uint16_t object_count;
  asurt_msgs__msg__ConeImg__Sequence imgs;
} asurt_msgs__msg__ConeImgArray;

// Struct for a sequence of asurt_msgs__msg__ConeImgArray.
typedef struct asurt_msgs__msg__ConeImgArray__Sequence
{
  asurt_msgs__msg__ConeImgArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asurt_msgs__msg__ConeImgArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__CONE_IMG_ARRAY__STRUCT_H_
