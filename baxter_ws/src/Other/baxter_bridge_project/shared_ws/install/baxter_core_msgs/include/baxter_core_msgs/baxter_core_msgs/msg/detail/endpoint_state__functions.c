// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/EndpointState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/endpoint_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `wrench`
#include "geometry_msgs/msg/detail/wrench__functions.h"

bool
baxter_core_msgs__msg__EndpointState__init(baxter_core_msgs__msg__EndpointState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    baxter_core_msgs__msg__EndpointState__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    baxter_core_msgs__msg__EndpointState__fini(msg);
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__init(&msg->twist)) {
    baxter_core_msgs__msg__EndpointState__fini(msg);
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__Wrench__init(&msg->wrench)) {
    baxter_core_msgs__msg__EndpointState__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__msg__EndpointState__fini(baxter_core_msgs__msg__EndpointState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // twist
  geometry_msgs__msg__Twist__fini(&msg->twist);
  // wrench
  geometry_msgs__msg__Wrench__fini(&msg->wrench);
}

bool
baxter_core_msgs__msg__EndpointState__are_equal(const baxter_core_msgs__msg__EndpointState * lhs, const baxter_core_msgs__msg__EndpointState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__are_equal(
      &(lhs->twist), &(rhs->twist)))
  {
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->wrench), &(rhs->wrench)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__EndpointState__copy(
  const baxter_core_msgs__msg__EndpointState * input,
  baxter_core_msgs__msg__EndpointState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // twist
  if (!geometry_msgs__msg__Twist__copy(
      &(input->twist), &(output->twist)))
  {
    return false;
  }
  // wrench
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->wrench), &(output->wrench)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__msg__EndpointState *
baxter_core_msgs__msg__EndpointState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndpointState * msg = (baxter_core_msgs__msg__EndpointState *)allocator.allocate(sizeof(baxter_core_msgs__msg__EndpointState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__EndpointState));
  bool success = baxter_core_msgs__msg__EndpointState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__EndpointState__destroy(baxter_core_msgs__msg__EndpointState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__EndpointState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__EndpointState__Sequence__init(baxter_core_msgs__msg__EndpointState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndpointState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__EndpointState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__EndpointState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__EndpointState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__EndpointState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__EndpointState__Sequence__fini(baxter_core_msgs__msg__EndpointState__Sequence * array)
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
      baxter_core_msgs__msg__EndpointState__fini(&array->data[i]);
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

baxter_core_msgs__msg__EndpointState__Sequence *
baxter_core_msgs__msg__EndpointState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndpointState__Sequence * array = (baxter_core_msgs__msg__EndpointState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__EndpointState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__EndpointState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__EndpointState__Sequence__destroy(baxter_core_msgs__msg__EndpointState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__EndpointState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__EndpointState__Sequence__are_equal(const baxter_core_msgs__msg__EndpointState__Sequence * lhs, const baxter_core_msgs__msg__EndpointState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__EndpointState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__EndpointState__Sequence__copy(
  const baxter_core_msgs__msg__EndpointState__Sequence * input,
  baxter_core_msgs__msg__EndpointState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__EndpointState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__EndpointState * data =
      (baxter_core_msgs__msg__EndpointState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__EndpointState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__EndpointState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__EndpointState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
