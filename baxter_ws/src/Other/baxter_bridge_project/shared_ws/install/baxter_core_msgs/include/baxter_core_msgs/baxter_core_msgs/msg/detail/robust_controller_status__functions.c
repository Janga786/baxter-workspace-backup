// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/RobustControllerStatus.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/robust_controller_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `control_uid`
// Member `error_codes`
// Member `labels`
#include "rosidl_runtime_c/string_functions.h"

bool
baxter_core_msgs__msg__RobustControllerStatus__init(baxter_core_msgs__msg__RobustControllerStatus * msg)
{
  if (!msg) {
    return false;
  }
  // is_enabled
  // complete
  // control_uid
  if (!rosidl_runtime_c__String__init(&msg->control_uid)) {
    baxter_core_msgs__msg__RobustControllerStatus__fini(msg);
    return false;
  }
  // timed_out
  // error_codes
  if (!rosidl_runtime_c__String__Sequence__init(&msg->error_codes, 0)) {
    baxter_core_msgs__msg__RobustControllerStatus__fini(msg);
    return false;
  }
  // labels
  if (!rosidl_runtime_c__String__Sequence__init(&msg->labels, 0)) {
    baxter_core_msgs__msg__RobustControllerStatus__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__msg__RobustControllerStatus__fini(baxter_core_msgs__msg__RobustControllerStatus * msg)
{
  if (!msg) {
    return;
  }
  // is_enabled
  // complete
  // control_uid
  rosidl_runtime_c__String__fini(&msg->control_uid);
  // timed_out
  // error_codes
  rosidl_runtime_c__String__Sequence__fini(&msg->error_codes);
  // labels
  rosidl_runtime_c__String__Sequence__fini(&msg->labels);
}

bool
baxter_core_msgs__msg__RobustControllerStatus__are_equal(const baxter_core_msgs__msg__RobustControllerStatus * lhs, const baxter_core_msgs__msg__RobustControllerStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_enabled
  if (lhs->is_enabled != rhs->is_enabled) {
    return false;
  }
  // complete
  if (lhs->complete != rhs->complete) {
    return false;
  }
  // control_uid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->control_uid), &(rhs->control_uid)))
  {
    return false;
  }
  // timed_out
  if (lhs->timed_out != rhs->timed_out) {
    return false;
  }
  // error_codes
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->error_codes), &(rhs->error_codes)))
  {
    return false;
  }
  // labels
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->labels), &(rhs->labels)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__RobustControllerStatus__copy(
  const baxter_core_msgs__msg__RobustControllerStatus * input,
  baxter_core_msgs__msg__RobustControllerStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // is_enabled
  output->is_enabled = input->is_enabled;
  // complete
  output->complete = input->complete;
  // control_uid
  if (!rosidl_runtime_c__String__copy(
      &(input->control_uid), &(output->control_uid)))
  {
    return false;
  }
  // timed_out
  output->timed_out = input->timed_out;
  // error_codes
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->error_codes), &(output->error_codes)))
  {
    return false;
  }
  // labels
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->labels), &(output->labels)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__msg__RobustControllerStatus *
baxter_core_msgs__msg__RobustControllerStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__RobustControllerStatus * msg = (baxter_core_msgs__msg__RobustControllerStatus *)allocator.allocate(sizeof(baxter_core_msgs__msg__RobustControllerStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__RobustControllerStatus));
  bool success = baxter_core_msgs__msg__RobustControllerStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__RobustControllerStatus__destroy(baxter_core_msgs__msg__RobustControllerStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__RobustControllerStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__RobustControllerStatus__Sequence__init(baxter_core_msgs__msg__RobustControllerStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__RobustControllerStatus * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__RobustControllerStatus *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__RobustControllerStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__RobustControllerStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__RobustControllerStatus__fini(&data[i - 1]);
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
baxter_core_msgs__msg__RobustControllerStatus__Sequence__fini(baxter_core_msgs__msg__RobustControllerStatus__Sequence * array)
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
      baxter_core_msgs__msg__RobustControllerStatus__fini(&array->data[i]);
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

baxter_core_msgs__msg__RobustControllerStatus__Sequence *
baxter_core_msgs__msg__RobustControllerStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__RobustControllerStatus__Sequence * array = (baxter_core_msgs__msg__RobustControllerStatus__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__RobustControllerStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__RobustControllerStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__RobustControllerStatus__Sequence__destroy(baxter_core_msgs__msg__RobustControllerStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__RobustControllerStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__RobustControllerStatus__Sequence__are_equal(const baxter_core_msgs__msg__RobustControllerStatus__Sequence * lhs, const baxter_core_msgs__msg__RobustControllerStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__RobustControllerStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__RobustControllerStatus__Sequence__copy(
  const baxter_core_msgs__msg__RobustControllerStatus__Sequence * input,
  baxter_core_msgs__msg__RobustControllerStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__RobustControllerStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__RobustControllerStatus * data =
      (baxter_core_msgs__msg__RobustControllerStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__RobustControllerStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__RobustControllerStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__RobustControllerStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
