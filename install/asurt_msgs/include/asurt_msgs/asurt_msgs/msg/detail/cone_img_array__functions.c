// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from asurt_msgs:msg/ConeImgArray.idl
// generated code does not contain a copyright notice
#include "asurt_msgs/msg/detail/cone_img_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `imgs`
#include "asurt_msgs/msg/detail/cone_img__functions.h"

bool
asurt_msgs__msg__ConeImgArray__init(asurt_msgs__msg__ConeImgArray * msg)
{
  if (!msg) {
    return false;
  }
  // view_id
  // object_count
  // imgs
  if (!asurt_msgs__msg__ConeImg__Sequence__init(&msg->imgs, 0)) {
    asurt_msgs__msg__ConeImgArray__fini(msg);
    return false;
  }
  return true;
}

void
asurt_msgs__msg__ConeImgArray__fini(asurt_msgs__msg__ConeImgArray * msg)
{
  if (!msg) {
    return;
  }
  // view_id
  // object_count
  // imgs
  asurt_msgs__msg__ConeImg__Sequence__fini(&msg->imgs);
}

bool
asurt_msgs__msg__ConeImgArray__are_equal(const asurt_msgs__msg__ConeImgArray * lhs, const asurt_msgs__msg__ConeImgArray * rhs)
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
  // imgs
  if (!asurt_msgs__msg__ConeImg__Sequence__are_equal(
      &(lhs->imgs), &(rhs->imgs)))
  {
    return false;
  }
  return true;
}

bool
asurt_msgs__msg__ConeImgArray__copy(
  const asurt_msgs__msg__ConeImgArray * input,
  asurt_msgs__msg__ConeImgArray * output)
{
  if (!input || !output) {
    return false;
  }
  // view_id
  output->view_id = input->view_id;
  // object_count
  output->object_count = input->object_count;
  // imgs
  if (!asurt_msgs__msg__ConeImg__Sequence__copy(
      &(input->imgs), &(output->imgs)))
  {
    return false;
  }
  return true;
}

asurt_msgs__msg__ConeImgArray *
asurt_msgs__msg__ConeImgArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__ConeImgArray * msg = (asurt_msgs__msg__ConeImgArray *)allocator.allocate(sizeof(asurt_msgs__msg__ConeImgArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(asurt_msgs__msg__ConeImgArray));
  bool success = asurt_msgs__msg__ConeImgArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
asurt_msgs__msg__ConeImgArray__destroy(asurt_msgs__msg__ConeImgArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    asurt_msgs__msg__ConeImgArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
asurt_msgs__msg__ConeImgArray__Sequence__init(asurt_msgs__msg__ConeImgArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__ConeImgArray * data = NULL;

  if (size) {
    data = (asurt_msgs__msg__ConeImgArray *)allocator.zero_allocate(size, sizeof(asurt_msgs__msg__ConeImgArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = asurt_msgs__msg__ConeImgArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        asurt_msgs__msg__ConeImgArray__fini(&data[i - 1]);
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
asurt_msgs__msg__ConeImgArray__Sequence__fini(asurt_msgs__msg__ConeImgArray__Sequence * array)
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
      asurt_msgs__msg__ConeImgArray__fini(&array->data[i]);
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

asurt_msgs__msg__ConeImgArray__Sequence *
asurt_msgs__msg__ConeImgArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  asurt_msgs__msg__ConeImgArray__Sequence * array = (asurt_msgs__msg__ConeImgArray__Sequence *)allocator.allocate(sizeof(asurt_msgs__msg__ConeImgArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = asurt_msgs__msg__ConeImgArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
asurt_msgs__msg__ConeImgArray__Sequence__destroy(asurt_msgs__msg__ConeImgArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    asurt_msgs__msg__ConeImgArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
asurt_msgs__msg__ConeImgArray__Sequence__are_equal(const asurt_msgs__msg__ConeImgArray__Sequence * lhs, const asurt_msgs__msg__ConeImgArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!asurt_msgs__msg__ConeImgArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
asurt_msgs__msg__ConeImgArray__Sequence__copy(
  const asurt_msgs__msg__ConeImgArray__Sequence * input,
  asurt_msgs__msg__ConeImgArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(asurt_msgs__msg__ConeImgArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    asurt_msgs__msg__ConeImgArray * data =
      (asurt_msgs__msg__ConeImgArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!asurt_msgs__msg__ConeImgArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          asurt_msgs__msg__ConeImgArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!asurt_msgs__msg__ConeImgArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
