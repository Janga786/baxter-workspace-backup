// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/NavigatorState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/navigator_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `button_names`
// Member `light_names`
#include "rosidl_runtime_c/string_functions.h"
// Member `buttons`
// Member `lights`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
baxter_core_msgs__msg__NavigatorState__init(baxter_core_msgs__msg__NavigatorState * msg)
{
  if (!msg) {
    return false;
  }
  // button_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->button_names, 0)) {
    baxter_core_msgs__msg__NavigatorState__fini(msg);
    return false;
  }
  // buttons
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->buttons, 0)) {
    baxter_core_msgs__msg__NavigatorState__fini(msg);
    return false;
  }
  // wheel
  // light_names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->light_names, 0)) {
    baxter_core_msgs__msg__NavigatorState__fini(msg);
    return false;
  }
  // lights
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->lights, 0)) {
    baxter_core_msgs__msg__NavigatorState__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__msg__NavigatorState__fini(baxter_core_msgs__msg__NavigatorState * msg)
{
  if (!msg) {
    return;
  }
  // button_names
  rosidl_runtime_c__String__Sequence__fini(&msg->button_names);
  // buttons
  rosidl_runtime_c__boolean__Sequence__fini(&msg->buttons);
  // wheel
  // light_names
  rosidl_runtime_c__String__Sequence__fini(&msg->light_names);
  // lights
  rosidl_runtime_c__boolean__Sequence__fini(&msg->lights);
}

bool
baxter_core_msgs__msg__NavigatorState__are_equal(const baxter_core_msgs__msg__NavigatorState * lhs, const baxter_core_msgs__msg__NavigatorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // button_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->button_names), &(rhs->button_names)))
  {
    return false;
  }
  // buttons
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->buttons), &(rhs->buttons)))
  {
    return false;
  }
  // wheel
  if (lhs->wheel != rhs->wheel) {
    return false;
  }
  // light_names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->light_names), &(rhs->light_names)))
  {
    return false;
  }
  // lights
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->lights), &(rhs->lights)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__NavigatorState__copy(
  const baxter_core_msgs__msg__NavigatorState * input,
  baxter_core_msgs__msg__NavigatorState * output)
{
  if (!input || !output) {
    return false;
  }
  // button_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->button_names), &(output->button_names)))
  {
    return false;
  }
  // buttons
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->buttons), &(output->buttons)))
  {
    return false;
  }
  // wheel
  output->wheel = input->wheel;
  // light_names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->light_names), &(output->light_names)))
  {
    return false;
  }
  // lights
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->lights), &(output->lights)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__msg__NavigatorState *
baxter_core_msgs__msg__NavigatorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__NavigatorState * msg = (baxter_core_msgs__msg__NavigatorState *)allocator.allocate(sizeof(baxter_core_msgs__msg__NavigatorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__NavigatorState));
  bool success = baxter_core_msgs__msg__NavigatorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__NavigatorState__destroy(baxter_core_msgs__msg__NavigatorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__NavigatorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__NavigatorState__Sequence__init(baxter_core_msgs__msg__NavigatorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__NavigatorState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__NavigatorState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__NavigatorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__NavigatorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__NavigatorState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__NavigatorState__Sequence__fini(baxter_core_msgs__msg__NavigatorState__Sequence * array)
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
      baxter_core_msgs__msg__NavigatorState__fini(&array->data[i]);
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

baxter_core_msgs__msg__NavigatorState__Sequence *
baxter_core_msgs__msg__NavigatorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__NavigatorState__Sequence * array = (baxter_core_msgs__msg__NavigatorState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__NavigatorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__NavigatorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__NavigatorState__Sequence__destroy(baxter_core_msgs__msg__NavigatorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__NavigatorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__NavigatorState__Sequence__are_equal(const baxter_core_msgs__msg__NavigatorState__Sequence * lhs, const baxter_core_msgs__msg__NavigatorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__NavigatorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__NavigatorState__Sequence__copy(
  const baxter_core_msgs__msg__NavigatorState__Sequence * input,
  baxter_core_msgs__msg__NavigatorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__NavigatorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__NavigatorState * data =
      (baxter_core_msgs__msg__NavigatorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__NavigatorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__NavigatorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__NavigatorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
