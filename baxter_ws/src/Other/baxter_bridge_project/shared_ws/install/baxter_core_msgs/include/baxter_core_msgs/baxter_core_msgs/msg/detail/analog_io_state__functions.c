// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/AnalogIOState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/analog_io_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
baxter_core_msgs__msg__AnalogIOState__init(baxter_core_msgs__msg__AnalogIOState * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    baxter_core_msgs__msg__AnalogIOState__fini(msg);
    return false;
  }
  // value
  // is_input_only
  return true;
}

void
baxter_core_msgs__msg__AnalogIOState__fini(baxter_core_msgs__msg__AnalogIOState * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // value
  // is_input_only
}

bool
baxter_core_msgs__msg__AnalogIOState__are_equal(const baxter_core_msgs__msg__AnalogIOState * lhs, const baxter_core_msgs__msg__AnalogIOState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // value
  if (lhs->value != rhs->value) {
    return false;
  }
  // is_input_only
  if (lhs->is_input_only != rhs->is_input_only) {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__AnalogIOState__copy(
  const baxter_core_msgs__msg__AnalogIOState * input,
  baxter_core_msgs__msg__AnalogIOState * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // value
  output->value = input->value;
  // is_input_only
  output->is_input_only = input->is_input_only;
  return true;
}

baxter_core_msgs__msg__AnalogIOState *
baxter_core_msgs__msg__AnalogIOState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__AnalogIOState * msg = (baxter_core_msgs__msg__AnalogIOState *)allocator.allocate(sizeof(baxter_core_msgs__msg__AnalogIOState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__AnalogIOState));
  bool success = baxter_core_msgs__msg__AnalogIOState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__AnalogIOState__destroy(baxter_core_msgs__msg__AnalogIOState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__AnalogIOState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__AnalogIOState__Sequence__init(baxter_core_msgs__msg__AnalogIOState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__AnalogIOState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__AnalogIOState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__AnalogIOState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__AnalogIOState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__AnalogIOState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__AnalogIOState__Sequence__fini(baxter_core_msgs__msg__AnalogIOState__Sequence * array)
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
      baxter_core_msgs__msg__AnalogIOState__fini(&array->data[i]);
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

baxter_core_msgs__msg__AnalogIOState__Sequence *
baxter_core_msgs__msg__AnalogIOState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__AnalogIOState__Sequence * array = (baxter_core_msgs__msg__AnalogIOState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__AnalogIOState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__AnalogIOState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__AnalogIOState__Sequence__destroy(baxter_core_msgs__msg__AnalogIOState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__AnalogIOState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__AnalogIOState__Sequence__are_equal(const baxter_core_msgs__msg__AnalogIOState__Sequence * lhs, const baxter_core_msgs__msg__AnalogIOState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__AnalogIOState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__AnalogIOState__Sequence__copy(
  const baxter_core_msgs__msg__AnalogIOState__Sequence * input,
  baxter_core_msgs__msg__AnalogIOState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__AnalogIOState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__AnalogIOState * data =
      (baxter_core_msgs__msg__AnalogIOState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__AnalogIOState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__AnalogIOState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__AnalogIOState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
