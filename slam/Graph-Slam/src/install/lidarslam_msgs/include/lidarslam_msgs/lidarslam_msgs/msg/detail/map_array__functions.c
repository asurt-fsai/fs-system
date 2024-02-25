// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidarslam_msgs:msg/MapArray.idl
// generated code does not contain a copyright notice
#include "lidarslam_msgs/msg/detail/map_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `submaps`
#include "lidarslam_msgs/msg/detail/sub_map__functions.h"

bool
lidarslam_msgs__msg__MapArray__init(lidarslam_msgs__msg__MapArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidarslam_msgs__msg__MapArray__fini(msg);
    return false;
  }
  // submaps
  if (!lidarslam_msgs__msg__SubMap__Sequence__init(&msg->submaps, 0)) {
    lidarslam_msgs__msg__MapArray__fini(msg);
    return false;
  }
  // cloud_coordinate
  return true;
}

void
lidarslam_msgs__msg__MapArray__fini(lidarslam_msgs__msg__MapArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // submaps
  lidarslam_msgs__msg__SubMap__Sequence__fini(&msg->submaps);
  // cloud_coordinate
}

bool
lidarslam_msgs__msg__MapArray__are_equal(const lidarslam_msgs__msg__MapArray * lhs, const lidarslam_msgs__msg__MapArray * rhs)
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
  // submaps
  if (!lidarslam_msgs__msg__SubMap__Sequence__are_equal(
      &(lhs->submaps), &(rhs->submaps)))
  {
    return false;
  }
  // cloud_coordinate
  if (lhs->cloud_coordinate != rhs->cloud_coordinate) {
    return false;
  }
  return true;
}

bool
lidarslam_msgs__msg__MapArray__copy(
  const lidarslam_msgs__msg__MapArray * input,
  lidarslam_msgs__msg__MapArray * output)
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
  // submaps
  if (!lidarslam_msgs__msg__SubMap__Sequence__copy(
      &(input->submaps), &(output->submaps)))
  {
    return false;
  }
  // cloud_coordinate
  output->cloud_coordinate = input->cloud_coordinate;
  return true;
}

lidarslam_msgs__msg__MapArray *
lidarslam_msgs__msg__MapArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidarslam_msgs__msg__MapArray * msg = (lidarslam_msgs__msg__MapArray *)allocator.allocate(sizeof(lidarslam_msgs__msg__MapArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidarslam_msgs__msg__MapArray));
  bool success = lidarslam_msgs__msg__MapArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidarslam_msgs__msg__MapArray__destroy(lidarslam_msgs__msg__MapArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidarslam_msgs__msg__MapArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidarslam_msgs__msg__MapArray__Sequence__init(lidarslam_msgs__msg__MapArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidarslam_msgs__msg__MapArray * data = NULL;

  if (size) {
    data = (lidarslam_msgs__msg__MapArray *)allocator.zero_allocate(size, sizeof(lidarslam_msgs__msg__MapArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidarslam_msgs__msg__MapArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidarslam_msgs__msg__MapArray__fini(&data[i - 1]);
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
lidarslam_msgs__msg__MapArray__Sequence__fini(lidarslam_msgs__msg__MapArray__Sequence * array)
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
      lidarslam_msgs__msg__MapArray__fini(&array->data[i]);
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

lidarslam_msgs__msg__MapArray__Sequence *
lidarslam_msgs__msg__MapArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidarslam_msgs__msg__MapArray__Sequence * array = (lidarslam_msgs__msg__MapArray__Sequence *)allocator.allocate(sizeof(lidarslam_msgs__msg__MapArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidarslam_msgs__msg__MapArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidarslam_msgs__msg__MapArray__Sequence__destroy(lidarslam_msgs__msg__MapArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidarslam_msgs__msg__MapArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidarslam_msgs__msg__MapArray__Sequence__are_equal(const lidarslam_msgs__msg__MapArray__Sequence * lhs, const lidarslam_msgs__msg__MapArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidarslam_msgs__msg__MapArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidarslam_msgs__msg__MapArray__Sequence__copy(
  const lidarslam_msgs__msg__MapArray__Sequence * input,
  lidarslam_msgs__msg__MapArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidarslam_msgs__msg__MapArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lidarslam_msgs__msg__MapArray * data =
      (lidarslam_msgs__msg__MapArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidarslam_msgs__msg__MapArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lidarslam_msgs__msg__MapArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidarslam_msgs__msg__MapArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
