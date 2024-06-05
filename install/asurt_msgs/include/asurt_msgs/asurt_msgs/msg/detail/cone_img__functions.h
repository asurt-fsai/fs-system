// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from asurt_msgs:msg/ConeImg.idl
// generated code does not contain a copyright notice

#ifndef ASURT_MSGS__MSG__DETAIL__CONE_IMG__FUNCTIONS_H_
#define ASURT_MSGS__MSG__DETAIL__CONE_IMG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "asurt_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "asurt_msgs/msg/detail/cone_img__struct.h"

/// Initialize msg/ConeImg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * asurt_msgs__msg__ConeImg
 * )) before or use
 * asurt_msgs__msg__ConeImg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
bool
asurt_msgs__msg__ConeImg__init(asurt_msgs__msg__ConeImg * msg);

/// Finalize msg/ConeImg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
void
asurt_msgs__msg__ConeImg__fini(asurt_msgs__msg__ConeImg * msg);

/// Create msg/ConeImg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * asurt_msgs__msg__ConeImg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
asurt_msgs__msg__ConeImg *
asurt_msgs__msg__ConeImg__create();

/// Destroy msg/ConeImg message.
/**
 * It calls
 * asurt_msgs__msg__ConeImg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
void
asurt_msgs__msg__ConeImg__destroy(asurt_msgs__msg__ConeImg * msg);

/// Check for msg/ConeImg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
bool
asurt_msgs__msg__ConeImg__are_equal(const asurt_msgs__msg__ConeImg * lhs, const asurt_msgs__msg__ConeImg * rhs);

/// Copy a msg/ConeImg message.
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
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
bool
asurt_msgs__msg__ConeImg__copy(
  const asurt_msgs__msg__ConeImg * input,
  asurt_msgs__msg__ConeImg * output);

/// Initialize array of msg/ConeImg messages.
/**
 * It allocates the memory for the number of elements and calls
 * asurt_msgs__msg__ConeImg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
bool
asurt_msgs__msg__ConeImg__Sequence__init(asurt_msgs__msg__ConeImg__Sequence * array, size_t size);

/// Finalize array of msg/ConeImg messages.
/**
 * It calls
 * asurt_msgs__msg__ConeImg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
void
asurt_msgs__msg__ConeImg__Sequence__fini(asurt_msgs__msg__ConeImg__Sequence * array);

/// Create array of msg/ConeImg messages.
/**
 * It allocates the memory for the array and calls
 * asurt_msgs__msg__ConeImg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
asurt_msgs__msg__ConeImg__Sequence *
asurt_msgs__msg__ConeImg__Sequence__create(size_t size);

/// Destroy array of msg/ConeImg messages.
/**
 * It calls
 * asurt_msgs__msg__ConeImg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
void
asurt_msgs__msg__ConeImg__Sequence__destroy(asurt_msgs__msg__ConeImg__Sequence * array);

/// Check for msg/ConeImg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
bool
asurt_msgs__msg__ConeImg__Sequence__are_equal(const asurt_msgs__msg__ConeImg__Sequence * lhs, const asurt_msgs__msg__ConeImg__Sequence * rhs);

/// Copy an array of msg/ConeImg messages.
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
ROSIDL_GENERATOR_C_PUBLIC_asurt_msgs
bool
asurt_msgs__msg__ConeImg__Sequence__copy(
  const asurt_msgs__msg__ConeImg__Sequence * input,
  asurt_msgs__msg__ConeImg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ASURT_MSGS__MSG__DETAIL__CONE_IMG__FUNCTIONS_H_
