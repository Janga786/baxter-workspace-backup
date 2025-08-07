// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/AssemblyState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/assembly_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
baxter_core_msgs__msg__AssemblyState__init(baxter_core_msgs__msg__AssemblyState * msg)
{
  if (!msg) {
    return false;
  }
  // ready
  // enabled
  // stopped
  // error
  // estop_button
  // estop_source
  return true;
}

void
baxter_core_msgs__msg__AssemblyState__fini(baxter_core_msgs__msg__AssemblyState * msg)
{
  if (!msg) {
    return;
  }
  // ready
  // enabled
  // stopped
  // error
  // estop_button
  // estop_source
}

bool
baxter_core_msgs__msg__AssemblyState__are_equal(const baxter_core_msgs__msg__AssemblyState * lhs, const baxter_core_msgs__msg__AssemblyState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ready
  if (lhs->ready != rhs->ready) {
    return false;
  }
  // enabled
  if (lhs->enabled != rhs->enabled) {
    return false;
  }
  // stopped
  if (lhs->stopped != rhs->stopped) {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  // estop_button
  if (lhs->estop_button != rhs->estop_button) {
    return false;
  }
  // estop_source
  if (lhs->estop_source != rhs->estop_source) {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__AssemblyState__copy(
  const baxter_core_msgs__msg__AssemblyState * input,
  baxter_core_msgs__msg__AssemblyState * output)
{
  if (!input || !output) {
    return false;
  }
  // ready
  output->ready = input->ready;
  // enabled
  output->enabled = input->enabled;
  // stopped
  output->stopped = input->stopped;
  // error
  output->error = input->error;
  // estop_button
  output->estop_button = input->estop_button;
  // estop_source
  output->estop_source = input->estop_source;
  return true;
}

baxter_core_msgs__msg__AssemblyState *
baxter_core_msgs__msg__AssemblyState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__AssemblyState * msg = (baxter_core_msgs__msg__AssemblyState *)allocator.allocate(sizeof(baxter_core_msgs__msg__AssemblyState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__AssemblyState));
  bool success = baxter_core_msgs__msg__AssemblyState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__AssemblyState__destroy(baxter_core_msgs__msg__AssemblyState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__AssemblyState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__AssemblyState__Sequence__init(baxter_core_msgs__msg__AssemblyState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__AssemblyState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__AssemblyState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__AssemblyState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__AssemblyState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__AssemblyState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__AssemblyState__Sequence__fini(baxter_core_msgs__msg__AssemblyState__Sequence * array)
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
      baxter_core_msgs__msg__AssemblyState__fini(&array->data[i]);
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

baxter_core_msgs__msg__AssemblyState__Sequence *
baxter_core_msgs__msg__AssemblyState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__AssemblyState__Sequence * array = (baxter_core_msgs__msg__AssemblyState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__AssemblyState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__AssemblyState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__AssemblyState__Sequence__destroy(baxter_core_msgs__msg__AssemblyState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__AssemblyState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__AssemblyState__Sequence__are_equal(const baxter_core_msgs__msg__AssemblyState__Sequence * lhs, const baxter_core_msgs__msg__AssemblyState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__AssemblyState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__AssemblyState__Sequence__copy(
  const baxter_core_msgs__msg__AssemblyState__Sequence * input,
  baxter_core_msgs__msg__AssemblyState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__AssemblyState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__AssemblyState * data =
      (baxter_core_msgs__msg__AssemblyState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__AssemblyState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__AssemblyState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__AssemblyState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
