// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asurt_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
#define ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BoundingBox in the package asurt_msgs.
typedef struct asurt_msgs__msg__BoundingBox
{
  float probability;
  uint16_t xmin;
  uint16_t ymin;
  uint16_t xmax;
  uint16_t ymax;
  uint16_t x_center;
  uint16_t y_center;
  uint16_t width;
  uint16_t height;
  uint16_t detection_id;
  uint16_t track_id;
  uint8_t type;
} asurt_msgs__msg__BoundingBox;

// Struct for a sequence of asurt_msgs__msg__BoundingBox.
typedef struct asurt_msgs__msg__BoundingBox__Sequence
{
  asurt_msgs__msg__BoundingBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asurt_msgs__msg__BoundingBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__BOUNDING_BOX__STRUCT_H_
