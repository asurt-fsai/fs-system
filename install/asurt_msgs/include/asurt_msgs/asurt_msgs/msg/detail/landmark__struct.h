// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asurt_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
#define ASURT_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'BLUE_CONE'.
enum
{
  asurt_msgs__msg__Landmark__BLUE_CONE = 0
};

/// Constant 'YELLOW_CONE'.
enum
{
  asurt_msgs__msg__Landmark__YELLOW_CONE = 1
};

/// Constant 'ORANGE_CONE'.
enum
{
  asurt_msgs__msg__Landmark__ORANGE_CONE = 2
};

/// Constant 'LARGE_CONE'.
enum
{
  asurt_msgs__msg__Landmark__LARGE_CONE = 3
};

/// Constant 'CONE_TYPE_UNKNOWN'.
enum
{
  asurt_msgs__msg__Landmark__CONE_TYPE_UNKNOWN = 4
};

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Landmark in the package asurt_msgs.
typedef struct asurt_msgs__msg__Landmark
{
  geometry_msgs__msg__Point position;
  uint32_t type;
  /// Used if there's a data association system available
  int32_t identifier;
  double probability;
} asurt_msgs__msg__Landmark;

// Struct for a sequence of asurt_msgs__msg__Landmark.
typedef struct asurt_msgs__msg__Landmark__Sequence
{
  asurt_msgs__msg__Landmark * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asurt_msgs__msg__Landmark__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
