// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from asurt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_H_
#define ASURT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STARTING'.
enum
{
  asurt_msgs__msg__NodeStatus__STARTING = 0
};

/// Constant 'READY'.
enum
{
  asurt_msgs__msg__NodeStatus__READY = 1
};

/// Constant 'RUNNING'.
enum
{
  asurt_msgs__msg__NodeStatus__RUNNING = 2
};

/// Constant 'ERROR'.
enum
{
  asurt_msgs__msg__NodeStatus__ERROR = 3
};

/// Constant 'SHUTDOWN'.
enum
{
  asurt_msgs__msg__NodeStatus__SHUTDOWN = 4
};

/// Constant 'UNRESPONSIVE'.
enum
{
  asurt_msgs__msg__NodeStatus__UNRESPONSIVE = 5
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/NodeStatus in the package asurt_msgs.
typedef struct asurt_msgs__msg__NodeStatus
{
  std_msgs__msg__Header header;
  uint8_t status;
  rosidl_runtime_c__String message;
} asurt_msgs__msg__NodeStatus;

// Struct for a sequence of asurt_msgs__msg__NodeStatus.
typedef struct asurt_msgs__msg__NodeStatus__Sequence
{
  asurt_msgs__msg__NodeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} asurt_msgs__msg__NodeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__NODE_STATUS__STRUCT_H_
