// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/URDFConfiguration.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/urdf_configuration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `time`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `link`
// Member `joint`
// Member `urdf`
#include "rosidl_runtime_c/string_functions.h"

bool
baxter_core_msgs__msg__URDFConfiguration__init(baxter_core_msgs__msg__URDFConfiguration * msg)
{
  if (!msg) {
    return false;
  }
  // time
  if (!builtin_interfaces__msg__Time__init(&msg->time)) {
    baxter_core_msgs__msg__URDFConfiguration__fini(msg);
    return false;
  }
  // link
  if (!rosidl_runtime_c__String__init(&msg->link)) {
    baxter_core_msgs__msg__URDFConfiguration__fini(msg);
    return false;
  }
  // joint
  if (!rosidl_runtime_c__String__init(&msg->joint)) {
    baxter_core_msgs__msg__URDFConfiguration__fini(msg);
    return false;
  }
  // urdf
  if (!rosidl_runtime_c__String__init(&msg->urdf)) {
    baxter_core_msgs__msg__URDFConfiguration__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__msg__URDFConfiguration__fini(baxter_core_msgs__msg__URDFConfiguration * msg)
{
  if (!msg) {
    return;
  }
  // time
  builtin_interfaces__msg__Time__fini(&msg->time);
  // link
  rosidl_runtime_c__String__fini(&msg->link);
  // joint
  rosidl_runtime_c__String__fini(&msg->joint);
  // urdf
  rosidl_runtime_c__String__fini(&msg->urdf);
}

bool
baxter_core_msgs__msg__URDFConfiguration__are_equal(const baxter_core_msgs__msg__URDFConfiguration * lhs, const baxter_core_msgs__msg__URDFConfiguration * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->time), &(rhs->time)))
  {
    return false;
  }
  // link
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->link), &(rhs->link)))
  {
    return false;
  }
  // joint
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->joint), &(rhs->joint)))
  {
    return false;
  }
  // urdf
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->urdf), &(rhs->urdf)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__URDFConfiguration__copy(
  const baxter_core_msgs__msg__URDFConfiguration * input,
  baxter_core_msgs__msg__URDFConfiguration * output)
{
  if (!input || !output) {
    return false;
  }
  // time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->time), &(output->time)))
  {
    return false;
  }
  // link
  if (!rosidl_runtime_c__String__copy(
      &(input->link), &(output->link)))
  {
    return false;
  }
  // joint
  if (!rosidl_runtime_c__String__copy(
      &(input->joint), &(output->joint)))
  {
    return false;
  }
  // urdf
  if (!rosidl_runtime_c__String__copy(
      &(input->urdf), &(output->urdf)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__msg__URDFConfiguration *
baxter_core_msgs__msg__URDFConfiguration__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__URDFConfiguration * msg = (baxter_core_msgs__msg__URDFConfiguration *)allocator.allocate(sizeof(baxter_core_msgs__msg__URDFConfiguration), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__URDFConfiguration));
  bool success = baxter_core_msgs__msg__URDFConfiguration__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__URDFConfiguration__destroy(baxter_core_msgs__msg__URDFConfiguration * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__URDFConfiguration__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__URDFConfiguration__Sequence__init(baxter_core_msgs__msg__URDFConfiguration__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__URDFConfiguration * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__URDFConfiguration *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__URDFConfiguration), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__URDFConfiguration__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__URDFConfiguration__fini(&data[i - 1]);
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
baxter_core_msgs__msg__URDFConfiguration__Sequence__fini(baxter_core_msgs__msg__URDFConfiguration__Sequence * array)
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
      baxter_core_msgs__msg__URDFConfiguration__fini(&array->data[i]);
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

baxter_core_msgs__msg__URDFConfiguration__Sequence *
baxter_core_msgs__msg__URDFConfiguration__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__URDFConfiguration__Sequence * array = (baxter_core_msgs__msg__URDFConfiguration__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__URDFConfiguration__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__URDFConfiguration__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__URDFConfiguration__Sequence__destroy(baxter_core_msgs__msg__URDFConfiguration__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__URDFConfiguration__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__URDFConfiguration__Sequence__are_equal(const baxter_core_msgs__msg__URDFConfiguration__Sequence * lhs, const baxter_core_msgs__msg__URDFConfiguration__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__URDFConfiguration__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__URDFConfiguration__Sequence__copy(
  const baxter_core_msgs__msg__URDFConfiguration__Sequence * input,
  baxter_core_msgs__msg__URDFConfiguration__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__URDFConfiguration);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__URDFConfiguration * data =
      (baxter_core_msgs__msg__URDFConfiguration *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__URDFConfiguration__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__URDFConfiguration__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__URDFConfiguration__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
