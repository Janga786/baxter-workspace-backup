// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/EndEffectorState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/end_effector_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `state`
// Member `command`
// Member `command_sender`
#include "rosidl_runtime_c/string_functions.h"

bool
baxter_core_msgs__msg__EndEffectorState__init(baxter_core_msgs__msg__EndEffectorState * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    baxter_core_msgs__msg__EndEffectorState__fini(msg);
    return false;
  }
  // id
  // enabled
  // calibrated
  // ready
  // moving
  // gripping
  // missed
  // error
  // reverse
  // position
  // force
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    baxter_core_msgs__msg__EndEffectorState__fini(msg);
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    baxter_core_msgs__msg__EndEffectorState__fini(msg);
    return false;
  }
  // command_sender
  if (!rosidl_runtime_c__String__init(&msg->command_sender)) {
    baxter_core_msgs__msg__EndEffectorState__fini(msg);
    return false;
  }
  // command_sequence
  return true;
}

void
baxter_core_msgs__msg__EndEffectorState__fini(baxter_core_msgs__msg__EndEffectorState * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // id
  // enabled
  // calibrated
  // ready
  // moving
  // gripping
  // missed
  // error
  // reverse
  // position
  // force
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // command
  rosidl_runtime_c__String__fini(&msg->command);
  // command_sender
  rosidl_runtime_c__String__fini(&msg->command_sender);
  // command_sequence
}

bool
baxter_core_msgs__msg__EndEffectorState__are_equal(const baxter_core_msgs__msg__EndEffectorState * lhs, const baxter_core_msgs__msg__EndEffectorState * rhs)
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
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // enabled
  if (lhs->enabled != rhs->enabled) {
    return false;
  }
  // calibrated
  if (lhs->calibrated != rhs->calibrated) {
    return false;
  }
  // ready
  if (lhs->ready != rhs->ready) {
    return false;
  }
  // moving
  if (lhs->moving != rhs->moving) {
    return false;
  }
  // gripping
  if (lhs->gripping != rhs->gripping) {
    return false;
  }
  // missed
  if (lhs->missed != rhs->missed) {
    return false;
  }
  // error
  if (lhs->error != rhs->error) {
    return false;
  }
  // reverse
  if (lhs->reverse != rhs->reverse) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  // force
  if (lhs->force != rhs->force) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  // command_sender
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command_sender), &(rhs->command_sender)))
  {
    return false;
  }
  // command_sequence
  if (lhs->command_sequence != rhs->command_sequence) {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__EndEffectorState__copy(
  const baxter_core_msgs__msg__EndEffectorState * input,
  baxter_core_msgs__msg__EndEffectorState * output)
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
  // id
  output->id = input->id;
  // enabled
  output->enabled = input->enabled;
  // calibrated
  output->calibrated = input->calibrated;
  // ready
  output->ready = input->ready;
  // moving
  output->moving = input->moving;
  // gripping
  output->gripping = input->gripping;
  // missed
  output->missed = input->missed;
  // error
  output->error = input->error;
  // reverse
  output->reverse = input->reverse;
  // position
  output->position = input->position;
  // force
  output->force = input->force;
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  // command_sender
  if (!rosidl_runtime_c__String__copy(
      &(input->command_sender), &(output->command_sender)))
  {
    return false;
  }
  // command_sequence
  output->command_sequence = input->command_sequence;
  return true;
}

baxter_core_msgs__msg__EndEffectorState *
baxter_core_msgs__msg__EndEffectorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndEffectorState * msg = (baxter_core_msgs__msg__EndEffectorState *)allocator.allocate(sizeof(baxter_core_msgs__msg__EndEffectorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__EndEffectorState));
  bool success = baxter_core_msgs__msg__EndEffectorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__EndEffectorState__destroy(baxter_core_msgs__msg__EndEffectorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__EndEffectorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__EndEffectorState__Sequence__init(baxter_core_msgs__msg__EndEffectorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndEffectorState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__EndEffectorState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__EndEffectorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__EndEffectorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__EndEffectorState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__EndEffectorState__Sequence__fini(baxter_core_msgs__msg__EndEffectorState__Sequence * array)
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
      baxter_core_msgs__msg__EndEffectorState__fini(&array->data[i]);
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

baxter_core_msgs__msg__EndEffectorState__Sequence *
baxter_core_msgs__msg__EndEffectorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndEffectorState__Sequence * array = (baxter_core_msgs__msg__EndEffectorState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__EndEffectorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__EndEffectorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__EndEffectorState__Sequence__destroy(baxter_core_msgs__msg__EndEffectorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__EndEffectorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__EndEffectorState__Sequence__are_equal(const baxter_core_msgs__msg__EndEffectorState__Sequence * lhs, const baxter_core_msgs__msg__EndEffectorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__EndEffectorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__EndEffectorState__Sequence__copy(
  const baxter_core_msgs__msg__EndEffectorState__Sequence * input,
  baxter_core_msgs__msg__EndEffectorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__EndEffectorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__EndEffectorState * data =
      (baxter_core_msgs__msg__EndEffectorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__EndEffectorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__EndEffectorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__EndEffectorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
