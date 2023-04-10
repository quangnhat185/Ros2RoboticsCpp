// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from pub_cpp:msg/SinAngle.idl
// generated code does not contain a copyright notice
#include "pub_cpp/msg/detail/sin_angle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
pub_cpp__msg__SinAngle__init(pub_cpp__msg__SinAngle * msg)
{
  if (!msg) {
    return false;
  }
  // angle
  // sin_angle
  return true;
}

void
pub_cpp__msg__SinAngle__fini(pub_cpp__msg__SinAngle * msg)
{
  if (!msg) {
    return;
  }
  // angle
  // sin_angle
}

bool
pub_cpp__msg__SinAngle__are_equal(const pub_cpp__msg__SinAngle * lhs, const pub_cpp__msg__SinAngle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // sin_angle
  if (lhs->sin_angle != rhs->sin_angle) {
    return false;
  }
  return true;
}

bool
pub_cpp__msg__SinAngle__copy(
  const pub_cpp__msg__SinAngle * input,
  pub_cpp__msg__SinAngle * output)
{
  if (!input || !output) {
    return false;
  }
  // angle
  output->angle = input->angle;
  // sin_angle
  output->sin_angle = input->sin_angle;
  return true;
}

pub_cpp__msg__SinAngle *
pub_cpp__msg__SinAngle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pub_cpp__msg__SinAngle * msg = (pub_cpp__msg__SinAngle *)allocator.allocate(sizeof(pub_cpp__msg__SinAngle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(pub_cpp__msg__SinAngle));
  bool success = pub_cpp__msg__SinAngle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
pub_cpp__msg__SinAngle__destroy(pub_cpp__msg__SinAngle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    pub_cpp__msg__SinAngle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
pub_cpp__msg__SinAngle__Sequence__init(pub_cpp__msg__SinAngle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pub_cpp__msg__SinAngle * data = NULL;

  if (size) {
    data = (pub_cpp__msg__SinAngle *)allocator.zero_allocate(size, sizeof(pub_cpp__msg__SinAngle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = pub_cpp__msg__SinAngle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        pub_cpp__msg__SinAngle__fini(&data[i - 1]);
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
pub_cpp__msg__SinAngle__Sequence__fini(pub_cpp__msg__SinAngle__Sequence * array)
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
      pub_cpp__msg__SinAngle__fini(&array->data[i]);
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

pub_cpp__msg__SinAngle__Sequence *
pub_cpp__msg__SinAngle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  pub_cpp__msg__SinAngle__Sequence * array = (pub_cpp__msg__SinAngle__Sequence *)allocator.allocate(sizeof(pub_cpp__msg__SinAngle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = pub_cpp__msg__SinAngle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
pub_cpp__msg__SinAngle__Sequence__destroy(pub_cpp__msg__SinAngle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    pub_cpp__msg__SinAngle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
pub_cpp__msg__SinAngle__Sequence__are_equal(const pub_cpp__msg__SinAngle__Sequence * lhs, const pub_cpp__msg__SinAngle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!pub_cpp__msg__SinAngle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
pub_cpp__msg__SinAngle__Sequence__copy(
  const pub_cpp__msg__SinAngle__Sequence * input,
  pub_cpp__msg__SinAngle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(pub_cpp__msg__SinAngle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    pub_cpp__msg__SinAngle * data =
      (pub_cpp__msg__SinAngle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!pub_cpp__msg__SinAngle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          pub_cpp__msg__SinAngle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!pub_cpp__msg__SinAngle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
