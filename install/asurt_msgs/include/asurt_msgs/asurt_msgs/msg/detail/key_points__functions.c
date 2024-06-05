// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asurt_msgs:msg/KeyPoints.idl
// generated code does not contain a copyright notice
#include "asurt_msgs/msg/detail/key_points__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `track_ids`
// Member `classes`
// Member `keypoints`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
asurt_msgs__msg__KeyPoints__init(asurt_msgs__msg__KeyPoints * msg)
{
  if (!msg) {
    return false;
  }
  // view_id
  // object_count
  // track_ids
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->track_ids, 0)) {
    asurt_msgs__msg__KeyPoints__fini(msg);
    return false;
  }
  // classes
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->classes, 0)) {
    asurt_msgs__msg__KeyPoints__fini(msg);
    return false;
  }
  // keypoints
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->keypoints, 0)) {
    asurt_msgs__msg__KeyPoints__fini(msg);
    return false;
  }
  return true;
}

void
asurt_msgs__msg__KeyPoints__fini(asurt_msgs__msg__KeyPoints * msg)
{
  if (!msg) {
    return;
  }
  // view_id
  // object_count
  // track_ids
  rosidl_runtime_c__uint8__Sequence__fini(&msg->track_ids);
  // classes
  rosidl_runtime_c__uint8__Sequence__fini(&msg->classes);
  // keypoints
  rosidl_runtime_c__uint16__Sequence__fini(&msg->keypoints);
}

bool
asurt_msgs__msg__KeyPoints__are_equal(const asurt_msgs__msg__KeyPoints * lhs, const asurt_msgs__msg__KeyPoints * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // view_id
  if (lhs->view_id != rhs->view_id) {
    return false;
  }
  // object_count
  if (lhs->object_count != rhs->object_count) {
    return false;
  }
  // track_ids
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->track_ids), &(rhs->track_ids)))
  {
    return false;
  }
  // classes
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->classes), &(rhs->classes)))
  {
    return false;
  }
  // keypoints
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->keypoints), &(rhs->keypoints)))
  {
    return false;
  }
  return true;
}

bool
asurt_msgs__msg__KeyPoints__copy(
  const asurt_msgs__msg__KeyPoints * input,
  asurt_msgs__msg__KeyPoints * output)
{
  if (!input || !output) {
    return false;
  }
  // view_id
  output->view_id = input->view_id;
  // object_count
  output->object_count = input->object_count;
  // track_ids
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->track_ids), &(output->track_ids)))
  {
    return false;
  }
  // classes
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->classes), &(output->classes)))
  {
    return false;
  }
  // keypoints
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->keypoints), &(output->keypoints)))
  {
    return false;
  }
  return true;
}

asurt_msgs__msg__KeyPoints *
asurt_msgs__msg__KeyPoints__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__KeyPoints * msg = (asurt_msgs__msg__KeyPoints *)allocator.allocate(sizeof(asurt_msgs__msg__KeyPoints), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asurt_msgs__msg__KeyPoints));
  bool success = asurt_msgs__msg__KeyPoints__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asurt_msgs__msg__KeyPoints__destroy(asurt_msgs__msg__KeyPoints * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asurt_msgs__msg__KeyPoints__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asurt_msgs__msg__KeyPoints__Sequence__init(asurt_msgs__msg__KeyPoints__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__KeyPoints * data = NULL;

  if (size) {
    data = (asurt_msgs__msg__KeyPoints *)allocator.zero_allocate(size, sizeof(asurt_msgs__msg__KeyPoints), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asurt_msgs__msg__KeyPoints__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asurt_msgs__msg__KeyPoints__fini(&data[i - 1]);
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
asurt_msgs__msg__KeyPoints__Sequence__fini(asurt_msgs__msg__KeyPoints__Sequence * array)
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
      asurt_msgs__msg__KeyPoints__fini(&array->data[i]);
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

asurt_msgs__msg__KeyPoints__Sequence *
asurt_msgs__msg__KeyPoints__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__KeyPoints__Sequence * array = (asurt_msgs__msg__KeyPoints__Sequence *)allocator.allocate(sizeof(asurt_msgs__msg__KeyPoints__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asurt_msgs__msg__KeyPoints__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asurt_msgs__msg__KeyPoints__Sequence__destroy(asurt_msgs__msg__KeyPoints__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asurt_msgs__msg__KeyPoints__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asurt_msgs__msg__KeyPoints__Sequence__are_equal(const asurt_msgs__msg__KeyPoints__Sequence * lhs, const asurt_msgs__msg__KeyPoints__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asurt_msgs__msg__KeyPoints__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asurt_msgs__msg__KeyPoints__Sequence__copy(
  const asurt_msgs__msg__KeyPoints__Sequence * input,
  asurt_msgs__msg__KeyPoints__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asurt_msgs__msg__KeyPoints);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asurt_msgs__msg__KeyPoints * data =
      (asurt_msgs__msg__KeyPoints *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asurt_msgs__msg__KeyPoints__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asurt_msgs__msg__KeyPoints__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asurt_msgs__msg__KeyPoints__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
