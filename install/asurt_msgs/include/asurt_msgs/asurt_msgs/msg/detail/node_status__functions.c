// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asurt_msgs:msg/NodeStatus.idl
// generated code does not contain a copyright notice
#include "asurt_msgs/msg/detail/node_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
asurt_msgs__msg__NodeStatus__init(asurt_msgs__msg__NodeStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    asurt_msgs__msg__NodeStatus__fini(msg);
    return false;
  }
  // status
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    asurt_msgs__msg__NodeStatus__fini(msg);
    return false;
  }
  return true;
}

void
asurt_msgs__msg__NodeStatus__fini(asurt_msgs__msg__NodeStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // status
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
asurt_msgs__msg__NodeStatus__are_equal(const asurt_msgs__msg__NodeStatus * lhs, const asurt_msgs__msg__NodeStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
asurt_msgs__msg__NodeStatus__copy(
  const asurt_msgs__msg__NodeStatus * input,
  asurt_msgs__msg__NodeStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // status
  output->status = input->status;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

asurt_msgs__msg__NodeStatus *
asurt_msgs__msg__NodeStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__NodeStatus * msg = (asurt_msgs__msg__NodeStatus *)allocator.allocate(sizeof(asurt_msgs__msg__NodeStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asurt_msgs__msg__NodeStatus));
  bool success = asurt_msgs__msg__NodeStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asurt_msgs__msg__NodeStatus__destroy(asurt_msgs__msg__NodeStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asurt_msgs__msg__NodeStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asurt_msgs__msg__NodeStatus__Sequence__init(asurt_msgs__msg__NodeStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__NodeStatus * data = NULL;

  if (size) {
    data = (asurt_msgs__msg__NodeStatus *)allocator.zero_allocate(size, sizeof(asurt_msgs__msg__NodeStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asurt_msgs__msg__NodeStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asurt_msgs__msg__NodeStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
asurt_msgs__msg__NodeStatus__Sequence__fini(asurt_msgs__msg__NodeStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      asurt_msgs__msg__NodeStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

asurt_msgs__msg__NodeStatus__Sequence *
asurt_msgs__msg__NodeStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__NodeStatus__Sequence * array = (asurt_msgs__msg__NodeStatus__Sequence *)allocator.allocate(sizeof(asurt_msgs__msg__NodeStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asurt_msgs__msg__NodeStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asurt_msgs__msg__NodeStatus__Sequence__destroy(asurt_msgs__msg__NodeStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asurt_msgs__msg__NodeStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asurt_msgs__msg__NodeStatus__Sequence__are_equal(const asurt_msgs__msg__NodeStatus__Sequence * lhs, const asurt_msgs__msg__NodeStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asurt_msgs__msg__NodeStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asurt_msgs__msg__NodeStatus__Sequence__copy(
  const asurt_msgs__msg__NodeStatus__Sequence * input,
  asurt_msgs__msg__NodeStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asurt_msgs__msg__NodeStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asurt_msgs__msg__NodeStatus * data =
      (asurt_msgs__msg__NodeStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asurt_msgs__msg__NodeStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asurt_msgs__msg__NodeStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asurt_msgs__msg__NodeStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
