// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lidarslam_msgs:msg/MapArray.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__FUNCTIONS_H_
#define LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lidarslam_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "lidarslam_msgs/msg/detail/map_array__struct.h"

/// Initialize msg/MapArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lidarslam_msgs__msg__MapArray
 * )) before or use
 * lidarslam_msgs__msg__MapArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__MapArray__init(lidarslam_msgs__msg__MapArray * msg);

/// Finalize msg/MapArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__MapArray__fini(lidarslam_msgs__msg__MapArray * msg);

/// Create msg/MapArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lidarslam_msgs__msg__MapArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
lidarslam_msgs__msg__MapArray *
lidarslam_msgs__msg__MapArray__create();

/// Destroy msg/MapArray message.
/**
 * It calls
 * lidarslam_msgs__msg__MapArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__MapArray__destroy(lidarslam_msgs__msg__MapArray * msg);

/// Check for msg/MapArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__MapArray__are_equal(const lidarslam_msgs__msg__MapArray * lhs, const lidarslam_msgs__msg__MapArray * rhs);

/// Copy a msg/MapArray message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__MapArray__copy(
  const lidarslam_msgs__msg__MapArray * input,
  lidarslam_msgs__msg__MapArray * output);

/// Initialize array of msg/MapArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * lidarslam_msgs__msg__MapArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__MapArray__Sequence__init(lidarslam_msgs__msg__MapArray__Sequence * array, size_t size);

/// Finalize array of msg/MapArray messages.
/**
 * It calls
 * lidarslam_msgs__msg__MapArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__MapArray__Sequence__fini(lidarslam_msgs__msg__MapArray__Sequence * array);

/// Create array of msg/MapArray messages.
/**
 * It allocates the memory for the array and calls
 * lidarslam_msgs__msg__MapArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
lidarslam_msgs__msg__MapArray__Sequence *
lidarslam_msgs__msg__MapArray__Sequence__create(size_t size);

/// Destroy array of msg/MapArray messages.
/**
 * It calls
 * lidarslam_msgs__msg__MapArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__MapArray__Sequence__destroy(lidarslam_msgs__msg__MapArray__Sequence * array);

/// Check for msg/MapArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__MapArray__Sequence__are_equal(const lidarslam_msgs__msg__MapArray__Sequence * lhs, const lidarslam_msgs__msg__MapArray__Sequence * rhs);

/// Copy an array of msg/MapArray messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__MapArray__Sequence__copy(
  const lidarslam_msgs__msg__MapArray__Sequence * input,
  lidarslam_msgs__msg__MapArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__MAP_ARRAY__FUNCTIONS_H_
