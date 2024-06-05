// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asurt_msgs:msg/ConeImg.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__CONE_IMG__STRUCT_H_
#define ASURT_MSGS__MSG__DETAIL__CONE_IMG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'img'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/ConeImg in the package asurt_msgs.
typedef struct asurt_msgs__msg__ConeImg
{
  uint16_t detection_id;
  uint16_t rows;
  uint16_t cols;
  sensor_msgs__msg__Image img;
  uint16_t track_id;
} asurt_msgs__msg__ConeImg;

// Struct for a sequence of asurt_msgs__msg__ConeImg.
typedef struct asurt_msgs__msg__ConeImg__Sequence
{
  asurt_msgs__msg__ConeImg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asurt_msgs__msg__ConeImg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__CONE_IMG__STRUCT_H_
