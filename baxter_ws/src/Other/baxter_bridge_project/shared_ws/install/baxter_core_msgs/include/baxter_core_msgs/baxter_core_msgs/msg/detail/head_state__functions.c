// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/HeadState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/head_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
baxter_core_msgs__msg__HeadState__init(baxter_core_msgs__msg__HeadState * msg)
{
  if (!msg) {
    return false;
  }
  // pan
  // is_turning
  // is_nodding
  // is_pan_enabled
  return true;
}

void
baxter_core_msgs__msg__HeadState__fini(baxter_core_msgs__msg__HeadState * msg)
{
  if (!msg) {
    return;
  }
  // pan
  // is_turning
  // is_nodding
  // is_pan_enabled
}

bool
baxter_core_msgs__msg__HeadState__are_equal(const baxter_core_msgs__msg__HeadState * lhs, const baxter_core_msgs__msg__HeadState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pan
  if (lhs->pan != rhs->pan) {
    return false;
  }
  // is_turning
  if (lhs->is_turning != rhs->is_turning) {
    return false;
  }
  // is_nodding
  if (lhs->is_nodding != rhs->is_nodding) {
    return false;
  }
  // is_pan_enabled
  if (lhs->is_pan_enabled != rhs->is_pan_enabled) {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__HeadState__copy(
  const baxter_core_msgs__msg__HeadState * input,
  baxter_core_msgs__msg__HeadState * output)
{
  if (!input || !output) {
    return false;
  }
  // pan
  output->pan = input->pan;
  // is_turning
  output->is_turning = input->is_turning;
  // is_nodding
  output->is_nodding = input->is_nodding;
  // is_pan_enabled
  output->is_pan_enabled = input->is_pan_enabled;
  return true;
}

baxter_core_msgs__msg__HeadState *
baxter_core_msgs__msg__HeadState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__HeadState * msg = (baxter_core_msgs__msg__HeadState *)allocator.allocate(sizeof(baxter_core_msgs__msg__HeadState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__HeadState));
  bool success = baxter_core_msgs__msg__HeadState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__HeadState__destroy(baxter_core_msgs__msg__HeadState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__HeadState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__HeadState__Sequence__init(baxter_core_msgs__msg__HeadState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__HeadState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__HeadState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__HeadState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__HeadState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__HeadState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__HeadState__Sequence__fini(baxter_core_msgs__msg__HeadState__Sequence * array)
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
      baxter_core_msgs__msg__HeadState__fini(&array->data[i]);
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

baxter_core_msgs__msg__HeadState__Sequence *
baxter_core_msgs__msg__HeadState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__HeadState__Sequence * array = (baxter_core_msgs__msg__HeadState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__HeadState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__HeadState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__HeadState__Sequence__destroy(baxter_core_msgs__msg__HeadState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__HeadState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__HeadState__Sequence__are_equal(const baxter_core_msgs__msg__HeadState__Sequence * lhs, const baxter_core_msgs__msg__HeadState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__HeadState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__HeadState__Sequence__copy(
  const baxter_core_msgs__msg__HeadState__Sequence * input,
  baxter_core_msgs__msg__HeadState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__HeadState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__HeadState * data =
      (baxter_core_msgs__msg__HeadState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__HeadState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__HeadState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__HeadState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
