// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidarslam_msgs:msg/SubMap.idl
// generated code does not contain a copyright notice
#include "lidarslam_msgs/msg/detail/sub_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `cloud`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
lidarslam_msgs__msg__SubMap__init(lidarslam_msgs__msg__SubMap * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidarslam_msgs__msg__SubMap__fini(msg);
    return false;
  }
  // distance
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    lidarslam_msgs__msg__SubMap__fini(msg);
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud)) {
    lidarslam_msgs__msg__SubMap__fini(msg);
    return false;
  }
  return true;
}

void
lidarslam_msgs__msg__SubMap__fini(lidarslam_msgs__msg__SubMap * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // distance
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // cloud
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud);
}

bool
lidarslam_msgs__msg__SubMap__are_equal(const lidarslam_msgs__msg__SubMap * lhs, const lidarslam_msgs__msg__SubMap * rhs)
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
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud), &(rhs->cloud)))
  {
    return false;
  }
  return true;
}

bool
lidarslam_msgs__msg__SubMap__copy(
  const lidarslam_msgs__msg__SubMap * input,
  lidarslam_msgs__msg__SubMap * output)
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
  // distance
  output->distance = input->distance;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // cloud
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud), &(output->cloud)))
  {
    return false;
  }
  return true;
}

lidarslam_msgs__msg__SubMap *
lidarslam_msgs__msg__SubMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidarslam_msgs__msg__SubMap * msg = (lidarslam_msgs__msg__SubMap *)allocator.allocate(sizeof(lidarslam_msgs__msg__SubMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidarslam_msgs__msg__SubMap));
  bool success = lidarslam_msgs__msg__SubMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidarslam_msgs__msg__SubMap__destroy(lidarslam_msgs__msg__SubMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidarslam_msgs__msg__SubMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidarslam_msgs__msg__SubMap__Sequence__init(lidarslam_msgs__msg__SubMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidarslam_msgs__msg__SubMap * data = NULL;

  if (size) {
    data = (lidarslam_msgs__msg__SubMap *)allocator.zero_allocate(size, sizeof(lidarslam_msgs__msg__SubMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidarslam_msgs__msg__SubMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidarslam_msgs__msg__SubMap__fini(&data[i - 1]);
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
lidarslam_msgs__msg__SubMap__Sequence__fini(lidarslam_msgs__msg__SubMap__Sequence * array)
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
      lidarslam_msgs__msg__SubMap__fini(&array->data[i]);
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

lidarslam_msgs__msg__SubMap__Sequence *
lidarslam_msgs__msg__SubMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidarslam_msgs__msg__SubMap__Sequence * array = (lidarslam_msgs__msg__SubMap__Sequence *)allocator.allocate(sizeof(lidarslam_msgs__msg__SubMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidarslam_msgs__msg__SubMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidarslam_msgs__msg__SubMap__Sequence__destroy(lidarslam_msgs__msg__SubMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidarslam_msgs__msg__SubMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidarslam_msgs__msg__SubMap__Sequence__are_equal(const lidarslam_msgs__msg__SubMap__Sequence * lhs, const lidarslam_msgs__msg__SubMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidarslam_msgs__msg__SubMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidarslam_msgs__msg__SubMap__Sequence__copy(
  const lidarslam_msgs__msg__SubMap__Sequence * input,
  lidarslam_msgs__msg__SubMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidarslam_msgs__msg__SubMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lidarslam_msgs__msg__SubMap * data =
      (lidarslam_msgs__msg__SubMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidarslam_msgs__msg__SubMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lidarslam_msgs__msg__SubMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidarslam_msgs__msg__SubMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
