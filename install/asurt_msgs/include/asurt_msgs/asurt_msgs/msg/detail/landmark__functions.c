// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asurt_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice
#include "asurt_msgs/msg/detail/landmark__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
asurt_msgs__msg__Landmark__init(asurt_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    asurt_msgs__msg__Landmark__fini(msg);
    return false;
  }
  // type
  // identifier
  // probability
  return true;
}

void
asurt_msgs__msg__Landmark__fini(asurt_msgs__msg__Landmark * msg)
{
  if (!msg) {
    return;
  }
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // type
  // identifier
  // probability
}

bool
asurt_msgs__msg__Landmark__are_equal(const asurt_msgs__msg__Landmark * lhs, const asurt_msgs__msg__Landmark * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // identifier
  if (lhs->identifier != rhs->identifier) {
    return false;
  }
  // probability
  if (lhs->probability != rhs->probability) {
    return false;
  }
  return true;
}

bool
asurt_msgs__msg__Landmark__copy(
  const asurt_msgs__msg__Landmark * input,
  asurt_msgs__msg__Landmark * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // type
  output->type = input->type;
  // identifier
  output->identifier = input->identifier;
  // probability
  output->probability = input->probability;
  return true;
}

asurt_msgs__msg__Landmark *
asurt_msgs__msg__Landmark__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__Landmark * msg = (asurt_msgs__msg__Landmark *)allocator.allocate(sizeof(asurt_msgs__msg__Landmark), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asurt_msgs__msg__Landmark));
  bool success = asurt_msgs__msg__Landmark__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asurt_msgs__msg__Landmark__destroy(asurt_msgs__msg__Landmark * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asurt_msgs__msg__Landmark__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asurt_msgs__msg__Landmark__Sequence__init(asurt_msgs__msg__Landmark__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__Landmark * data = NULL;

  if (size) {
    data = (asurt_msgs__msg__Landmark *)allocator.zero_allocate(size, sizeof(asurt_msgs__msg__Landmark), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asurt_msgs__msg__Landmark__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asurt_msgs__msg__Landmark__fini(&data[i - 1]);
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
asurt_msgs__msg__Landmark__Sequence__fini(asurt_msgs__msg__Landmark__Sequence * array)
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
      asurt_msgs__msg__Landmark__fini(&array->data[i]);
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

asurt_msgs__msg__Landmark__Sequence *
asurt_msgs__msg__Landmark__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__Landmark__Sequence * array = (asurt_msgs__msg__Landmark__Sequence *)allocator.allocate(sizeof(asurt_msgs__msg__Landmark__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asurt_msgs__msg__Landmark__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asurt_msgs__msg__Landmark__Sequence__destroy(asurt_msgs__msg__Landmark__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asurt_msgs__msg__Landmark__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asurt_msgs__msg__Landmark__Sequence__are_equal(const asurt_msgs__msg__Landmark__Sequence * lhs, const asurt_msgs__msg__Landmark__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asurt_msgs__msg__Landmark__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asurt_msgs__msg__Landmark__Sequence__copy(
  const asurt_msgs__msg__Landmark__Sequence * input,
  asurt_msgs__msg__Landmark__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asurt_msgs__msg__Landmark);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asurt_msgs__msg__Landmark * data =
      (asurt_msgs__msg__Landmark *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asurt_msgs__msg__Landmark__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asurt_msgs__msg__Landmark__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asurt_msgs__msg__Landmark__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
