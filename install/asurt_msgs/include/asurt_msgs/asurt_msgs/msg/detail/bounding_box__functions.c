// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asurt_msgs:msg/BoundingBox.idl
// generated code does not contain a copyright notice
#include "asurt_msgs/msg/detail/bounding_box__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
asurt_msgs__msg__BoundingBox__init(asurt_msgs__msg__BoundingBox * msg)
{
  if (!msg) {
    return false;
  }
  // probability
  // xmin
  // ymin
  // xmax
  // ymax
  // x_center
  // y_center
  // width
  // height
  // detection_id
  // track_id
  // type
  return true;
}

void
asurt_msgs__msg__BoundingBox__fini(asurt_msgs__msg__BoundingBox * msg)
{
  if (!msg) {
    return;
  }
  // probability
  // xmin
  // ymin
  // xmax
  // ymax
  // x_center
  // y_center
  // width
  // height
  // detection_id
  // track_id
  // type
}

bool
asurt_msgs__msg__BoundingBox__are_equal(const asurt_msgs__msg__BoundingBox * lhs, const asurt_msgs__msg__BoundingBox * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // probability
  if (lhs->probability != rhs->probability) {
    return false;
  }
  // xmin
  if (lhs->xmin != rhs->xmin) {
    return false;
  }
  // ymin
  if (lhs->ymin != rhs->ymin) {
    return false;
  }
  // xmax
  if (lhs->xmax != rhs->xmax) {
    return false;
  }
  // ymax
  if (lhs->ymax != rhs->ymax) {
    return false;
  }
  // x_center
  if (lhs->x_center != rhs->x_center) {
    return false;
  }
  // y_center
  if (lhs->y_center != rhs->y_center) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // detection_id
  if (lhs->detection_id != rhs->detection_id) {
    return false;
  }
  // track_id
  if (lhs->track_id != rhs->track_id) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  return true;
}

bool
asurt_msgs__msg__BoundingBox__copy(
  const asurt_msgs__msg__BoundingBox * input,
  asurt_msgs__msg__BoundingBox * output)
{
  if (!input || !output) {
    return false;
  }
  // probability
  output->probability = input->probability;
  // xmin
  output->xmin = input->xmin;
  // ymin
  output->ymin = input->ymin;
  // xmax
  output->xmax = input->xmax;
  // ymax
  output->ymax = input->ymax;
  // x_center
  output->x_center = input->x_center;
  // y_center
  output->y_center = input->y_center;
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // detection_id
  output->detection_id = input->detection_id;
  // track_id
  output->track_id = input->track_id;
  // type
  output->type = input->type;
  return true;
}

asurt_msgs__msg__BoundingBox *
asurt_msgs__msg__BoundingBox__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__BoundingBox * msg = (asurt_msgs__msg__BoundingBox *)allocator.allocate(sizeof(asurt_msgs__msg__BoundingBox), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asurt_msgs__msg__BoundingBox));
  bool success = asurt_msgs__msg__BoundingBox__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asurt_msgs__msg__BoundingBox__destroy(asurt_msgs__msg__BoundingBox * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asurt_msgs__msg__BoundingBox__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asurt_msgs__msg__BoundingBox__Sequence__init(asurt_msgs__msg__BoundingBox__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__BoundingBox * data = NULL;

  if (size) {
    data = (asurt_msgs__msg__BoundingBox *)allocator.zero_allocate(size, sizeof(asurt_msgs__msg__BoundingBox), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asurt_msgs__msg__BoundingBox__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asurt_msgs__msg__BoundingBox__fini(&data[i - 1]);
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
asurt_msgs__msg__BoundingBox__Sequence__fini(asurt_msgs__msg__BoundingBox__Sequence * array)
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
      asurt_msgs__msg__BoundingBox__fini(&array->data[i]);
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

asurt_msgs__msg__BoundingBox__Sequence *
asurt_msgs__msg__BoundingBox__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__BoundingBox__Sequence * array = (asurt_msgs__msg__BoundingBox__Sequence *)allocator.allocate(sizeof(asurt_msgs__msg__BoundingBox__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asurt_msgs__msg__BoundingBox__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asurt_msgs__msg__BoundingBox__Sequence__destroy(asurt_msgs__msg__BoundingBox__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asurt_msgs__msg__BoundingBox__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asurt_msgs__msg__BoundingBox__Sequence__are_equal(const asurt_msgs__msg__BoundingBox__Sequence * lhs, const asurt_msgs__msg__BoundingBox__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asurt_msgs__msg__BoundingBox__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asurt_msgs__msg__BoundingBox__Sequence__copy(
  const asurt_msgs__msg__BoundingBox__Sequence * input,
  asurt_msgs__msg__BoundingBox__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asurt_msgs__msg__BoundingBox);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asurt_msgs__msg__BoundingBox * data =
      (asurt_msgs__msg__BoundingBox *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asurt_msgs__msg__BoundingBox__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asurt_msgs__msg__BoundingBox__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asurt_msgs__msg__BoundingBox__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
