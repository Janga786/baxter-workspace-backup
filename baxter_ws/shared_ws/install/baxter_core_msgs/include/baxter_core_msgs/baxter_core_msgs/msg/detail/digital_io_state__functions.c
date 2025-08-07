// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/DigitalIOState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/digital_io_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
baxter_core_msgs__msg__DigitalIOState__init(baxter_core_msgs__msg__DigitalIOState * msg)
{
  if (!msg) {
    return false;
  }
  // state
  // is_input_only
  return true;
}

void
baxter_core_msgs__msg__DigitalIOState__fini(baxter_core_msgs__msg__DigitalIOState * msg)
{
  if (!msg) {
    return;
  }
  // state
  // is_input_only
}

bool
baxter_core_msgs__msg__DigitalIOState__are_equal(const baxter_core_msgs__msg__DigitalIOState * lhs, const baxter_core_msgs__msg__DigitalIOState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // is_input_only
  if (lhs->is_input_only != rhs->is_input_only) {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__DigitalIOState__copy(
  const baxter_core_msgs__msg__DigitalIOState * input,
  baxter_core_msgs__msg__DigitalIOState * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  output->state = input->state;
  // is_input_only
  output->is_input_only = input->is_input_only;
  return true;
}

baxter_core_msgs__msg__DigitalIOState *
baxter_core_msgs__msg__DigitalIOState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__DigitalIOState * msg = (baxter_core_msgs__msg__DigitalIOState *)allocator.allocate(sizeof(baxter_core_msgs__msg__DigitalIOState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__DigitalIOState));
  bool success = baxter_core_msgs__msg__DigitalIOState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__DigitalIOState__destroy(baxter_core_msgs__msg__DigitalIOState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__DigitalIOState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__DigitalIOState__Sequence__init(baxter_core_msgs__msg__DigitalIOState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__DigitalIOState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__DigitalIOState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__DigitalIOState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__DigitalIOState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__DigitalIOState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__DigitalIOState__Sequence__fini(baxter_core_msgs__msg__DigitalIOState__Sequence * array)
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
      baxter_core_msgs__msg__DigitalIOState__fini(&array->data[i]);
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

baxter_core_msgs__msg__DigitalIOState__Sequence *
baxter_core_msgs__msg__DigitalIOState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__DigitalIOState__Sequence * array = (baxter_core_msgs__msg__DigitalIOState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__DigitalIOState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__DigitalIOState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__DigitalIOState__Sequence__destroy(baxter_core_msgs__msg__DigitalIOState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__DigitalIOState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__DigitalIOState__Sequence__are_equal(const baxter_core_msgs__msg__DigitalIOState__Sequence * lhs, const baxter_core_msgs__msg__DigitalIOState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__DigitalIOState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__DigitalIOState__Sequence__copy(
  const baxter_core_msgs__msg__DigitalIOState__Sequence * input,
  baxter_core_msgs__msg__DigitalIOState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__DigitalIOState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__DigitalIOState * data =
      (baxter_core_msgs__msg__DigitalIOState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__DigitalIOState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__DigitalIOState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__DigitalIOState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
