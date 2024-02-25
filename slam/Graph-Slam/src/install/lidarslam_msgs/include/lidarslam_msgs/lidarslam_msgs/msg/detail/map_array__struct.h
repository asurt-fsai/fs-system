// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidarslam_msgs:msg/MapArray.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__STRUCT_H_
#define LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LOCAL'.
enum
{
  lidarslam_msgs__msg__MapArray__LOCAL = 0
};

/// Constant 'GLOBAL'.
enum
{
  lidarslam_msgs__msg__MapArray__GLOBAL = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'submaps'
#include "lidarslam_msgs/msg/detail/sub_map__struct.h"

/// Struct defined in msg/MapArray in the package lidarslam_msgs.
typedef struct lidarslam_msgs__msg__MapArray
{
  std_msgs__msg__Header header;
  lidarslam_msgs__msg__SubMap__Sequence submaps;
  int8_t cloud_coordinate;
} lidarslam_msgs__msg__MapArray;

// Struct for a sequence of lidarslam_msgs__msg__MapArray.
typedef struct lidarslam_msgs__msg__MapArray__Sequence
{
  lidarslam_msgs__msg__MapArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidarslam_msgs__msg__MapArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__STRUCT_H_
