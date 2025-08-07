// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_maintenance_msgs:msg/UpdateStatus.idl
// generated code does not contain a copyright notice
#include "baxter_maintenance_msgs/msg/detail/update_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `long_description`
#include "rosidl_runtime_c/string_functions.h"

bool
baxter_maintenance_msgs__msg__UpdateStatus__init(baxter_maintenance_msgs__msg__UpdateStatus * msg)
{
  if (!msg) {
    return false;
  }
  // status
  // progress
  // long_description
  if (!rosidl_runtime_c__String__init(&msg->long_description)) {
    baxter_maintenance_msgs__msg__UpdateStatus__fini(msg);
    return false;
  }
  return true;
}

void
baxter_maintenance_msgs__msg__UpdateStatus__fini(baxter_maintenance_msgs__msg__UpdateStatus * msg)
{
  if (!msg) {
    return;
  }
  // status
  // progress
  // long_description
  rosidl_runtime_c__String__fini(&msg->long_description);
}

bool
baxter_maintenance_msgs__msg__UpdateStatus__are_equal(const baxter_maintenance_msgs__msg__UpdateStatus * lhs, const baxter_maintenance_msgs__msg__UpdateStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  // progress
  if (lhs->progress != rhs->progress) {
    return false;
  }
  // long_description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->long_description), &(rhs->long_description)))
  {
    return false;
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__UpdateStatus__copy(
  const baxter_maintenance_msgs__msg__UpdateStatus * input,
  baxter_maintenance_msgs__msg__UpdateStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // status
  output->status = input->status;
  // progress
  output->progress = input->progress;
  // long_description
  if (!rosidl_runtime_c__String__copy(
      &(input->long_description), &(output->long_description)))
  {
    return false;
  }
  return true;
}

baxter_maintenance_msgs__msg__UpdateStatus *
baxter_maintenance_msgs__msg__UpdateStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__UpdateStatus * msg = (baxter_maintenance_msgs__msg__UpdateStatus *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__UpdateStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_maintenance_msgs__msg__UpdateStatus));
  bool success = baxter_maintenance_msgs__msg__UpdateStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_maintenance_msgs__msg__UpdateStatus__destroy(baxter_maintenance_msgs__msg__UpdateStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_maintenance_msgs__msg__UpdateStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_maintenance_msgs__msg__UpdateStatus__Sequence__init(baxter_maintenance_msgs__msg__UpdateStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__UpdateStatus * data = NULL;

  if (size) {
    data = (baxter_maintenance_msgs__msg__UpdateStatus *)allocator.zero_allocate(size, sizeof(baxter_maintenance_msgs__msg__UpdateStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_maintenance_msgs__msg__UpdateStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_maintenance_msgs__msg__UpdateStatus__fini(&data[i - 1]);
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
baxter_maintenance_msgs__msg__UpdateStatus__Sequence__fini(baxter_maintenance_msgs__msg__UpdateStatus__Sequence * array)
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
      baxter_maintenance_msgs__msg__UpdateStatus__fini(&array->data[i]);
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

baxter_maintenance_msgs__msg__UpdateStatus__Sequence *
baxter_maintenance_msgs__msg__UpdateStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__UpdateStatus__Sequence * array = (baxter_maintenance_msgs__msg__UpdateStatus__Sequence *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__UpdateStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_maintenance_msgs__msg__UpdateStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_maintenance_msgs__msg__UpdateStatus__Sequence__destroy(baxter_maintenance_msgs__msg__UpdateStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_maintenance_msgs__msg__UpdateStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_maintenance_msgs__msg__UpdateStatus__Sequence__are_equal(const baxter_maintenance_msgs__msg__UpdateStatus__Sequence * lhs, const baxter_maintenance_msgs__msg__UpdateStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_maintenance_msgs__msg__UpdateStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__UpdateStatus__Sequence__copy(
  const baxter_maintenance_msgs__msg__UpdateStatus__Sequence * input,
  baxter_maintenance_msgs__msg__UpdateStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_maintenance_msgs__msg__UpdateStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_maintenance_msgs__msg__UpdateStatus * data =
      (baxter_maintenance_msgs__msg__UpdateStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_maintenance_msgs__msg__UpdateStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_maintenance_msgs__msg__UpdateStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_maintenance_msgs__msg__UpdateStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
