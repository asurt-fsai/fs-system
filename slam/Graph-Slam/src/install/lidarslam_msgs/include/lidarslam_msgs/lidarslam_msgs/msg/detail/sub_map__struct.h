// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidarslam_msgs:msg/SubMap.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__STRUCT_H_
#define LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'cloud'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in msg/SubMap in the package lidarslam_msgs.
typedef struct lidarslam_msgs__msg__SubMap
{
  std_msgs__msg__Header header;
  double distance;
  geometry_msgs__msg__Pose pose;
  sensor_msgs__msg__PointCloud2 cloud;
} lidarslam_msgs__msg__SubMap;

// Struct for a sequence of lidarslam_msgs__msg__SubMap.
typedef struct lidarslam_msgs__msg__SubMap__Sequence
{
  lidarslam_msgs__msg__SubMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidarslam_msgs__msg__SubMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__STRUCT_H_
