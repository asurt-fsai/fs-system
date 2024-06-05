// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asurt_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_H_
#define ASURT_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'track_ids'
// Member 'classes'
// Member 'keypoints'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/KeyPoints in the package asurt_msgs.
typedef struct asurt_msgs__msg__KeyPoints
{
  uint32_t view_id;
  uint16_t object_count;
  rosidl_runtime_c__uint8__Sequence track_ids;
  rosidl_runtime_c__uint8__Sequence classes;
  rosidl_runtime_c__uint16__Sequence keypoints;
} asurt_msgs__msg__KeyPoints;

// Struct for a sequence of asurt_msgs__msg__KeyPoints.
typedef struct asurt_msgs__msg__KeyPoints__Sequence
{
  asurt_msgs__msg__KeyPoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asurt_msgs__msg__KeyPoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__KEY_POINTS__STRUCT_H_
