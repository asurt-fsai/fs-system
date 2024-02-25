// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lidarslam_msgs:msg/SubMap.idl
// generated code does not contain a copyright notice

#ifndef LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__FUNCTIONS_H_
#define LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lidarslam_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "lidarslam_msgs/msg/detail/sub_map__struct.h"

/// Initialize msg/SubMap message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lidarslam_msgs__msg__SubMap
 * )) before or use
 * lidarslam_msgs__msg__SubMap__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__SubMap__init(lidarslam_msgs__msg__SubMap * msg);

/// Finalize msg/SubMap message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__SubMap__fini(lidarslam_msgs__msg__SubMap * msg);

/// Create msg/SubMap message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lidarslam_msgs__msg__SubMap__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
lidarslam_msgs__msg__SubMap *
lidarslam_msgs__msg__SubMap__create();

/// Destroy msg/SubMap message.
/**
 * It calls
 * lidarslam_msgs__msg__SubMap__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__SubMap__destroy(lidarslam_msgs__msg__SubMap * msg);

/// Check for msg/SubMap message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__SubMap__are_equal(const lidarslam_msgs__msg__SubMap * lhs, const lidarslam_msgs__msg__SubMap * rhs);

/// Copy a msg/SubMap message.
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
lidarslam_msgs__msg__SubMap__copy(
  const lidarslam_msgs__msg__SubMap * input,
  lidarslam_msgs__msg__SubMap * output);

/// Initialize array of msg/SubMap messages.
/**
 * It allocates the memory for the number of elements and calls
 * lidarslam_msgs__msg__SubMap__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__SubMap__Sequence__init(lidarslam_msgs__msg__SubMap__Sequence * array, size_t size);

/// Finalize array of msg/SubMap messages.
/**
 * It calls
 * lidarslam_msgs__msg__SubMap__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__SubMap__Sequence__fini(lidarslam_msgs__msg__SubMap__Sequence * array);

/// Create array of msg/SubMap messages.
/**
 * It allocates the memory for the array and calls
 * lidarslam_msgs__msg__SubMap__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
lidarslam_msgs__msg__SubMap__Sequence *
lidarslam_msgs__msg__SubMap__Sequence__create(size_t size);

/// Destroy array of msg/SubMap messages.
/**
 * It calls
 * lidarslam_msgs__msg__SubMap__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
void
lidarslam_msgs__msg__SubMap__Sequence__destroy(lidarslam_msgs__msg__SubMap__Sequence * array);

/// Check for msg/SubMap message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidarslam_msgs
bool
lidarslam_msgs__msg__SubMap__Sequence__are_equal(const lidarslam_msgs__msg__SubMap__Sequence * lhs, const lidarslam_msgs__msg__SubMap__Sequence * rhs);

/// Copy an array of msg/SubMap messages.
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
lidarslam_msgs__msg__SubMap__Sequence__copy(
  const lidarslam_msgs__msg__SubMap__Sequence * input,
  lidarslam_msgs__msg__SubMap__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LIDARSLAM_MSGS__MSG__DETAIL__SUB_MAP__FUNCTIONS_H_
