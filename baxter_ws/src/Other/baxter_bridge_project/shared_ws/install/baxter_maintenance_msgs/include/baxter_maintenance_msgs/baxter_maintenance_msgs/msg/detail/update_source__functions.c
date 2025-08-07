// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_maintenance_msgs:msg/UpdateSource.idl
// generated code does not contain a copyright notice
#include "baxter_maintenance_msgs/msg/detail/update_source__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `devname`
// Member `filename`
// Member `version`
// Member `uuid`
#include "rosidl_runtime_c/string_functions.h"

bool
baxter_maintenance_msgs__msg__UpdateSource__init(baxter_maintenance_msgs__msg__UpdateSource * msg)
{
  if (!msg) {
    return false;
  }
  // devname
  if (!rosidl_runtime_c__String__init(&msg->devname)) {
    baxter_maintenance_msgs__msg__UpdateSource__fini(msg);
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__init(&msg->filename)) {
    baxter_maintenance_msgs__msg__UpdateSource__fini(msg);
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__init(&msg->version)) {
    baxter_maintenance_msgs__msg__UpdateSource__fini(msg);
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__init(&msg->uuid)) {
    baxter_maintenance_msgs__msg__UpdateSource__fini(msg);
    return false;
  }
  return true;
}

void
baxter_maintenance_msgs__msg__UpdateSource__fini(baxter_maintenance_msgs__msg__UpdateSource * msg)
{
  if (!msg) {
    return;
  }
  // devname
  rosidl_runtime_c__String__fini(&msg->devname);
  // filename
  rosidl_runtime_c__String__fini(&msg->filename);
  // version
  rosidl_runtime_c__String__fini(&msg->version);
  // uuid
  rosidl_runtime_c__String__fini(&msg->uuid);
}

bool
baxter_maintenance_msgs__msg__UpdateSource__are_equal(const baxter_maintenance_msgs__msg__UpdateSource * lhs, const baxter_maintenance_msgs__msg__UpdateSource * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // devname
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->devname), &(rhs->devname)))
  {
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->filename), &(rhs->filename)))
  {
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->version), &(rhs->version)))
  {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->uuid), &(rhs->uuid)))
  {
    return false;
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__UpdateSource__copy(
  const baxter_maintenance_msgs__msg__UpdateSource * input,
  baxter_maintenance_msgs__msg__UpdateSource * output)
{
  if (!input || !output) {
    return false;
  }
  // devname
  if (!rosidl_runtime_c__String__copy(
      &(input->devname), &(output->devname)))
  {
    return false;
  }
  // filename
  if (!rosidl_runtime_c__String__copy(
      &(input->filename), &(output->filename)))
  {
    return false;
  }
  // version
  if (!rosidl_runtime_c__String__copy(
      &(input->version), &(output->version)))
  {
    return false;
  }
  // uuid
  if (!rosidl_runtime_c__String__copy(
      &(input->uuid), &(output->uuid)))
  {
    return false;
  }
  return true;
}

baxter_maintenance_msgs__msg__UpdateSource *
baxter_maintenance_msgs__msg__UpdateSource__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__UpdateSource * msg = (baxter_maintenance_msgs__msg__UpdateSource *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__UpdateSource), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_maintenance_msgs__msg__UpdateSource));
  bool success = baxter_maintenance_msgs__msg__UpdateSource__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_maintenance_msgs__msg__UpdateSource__destroy(baxter_maintenance_msgs__msg__UpdateSource * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_maintenance_msgs__msg__UpdateSource__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_maintenance_msgs__msg__UpdateSource__Sequence__init(baxter_maintenance_msgs__msg__UpdateSource__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__UpdateSource * data = NULL;

  if (size) {
    data = (baxter_maintenance_msgs__msg__UpdateSource *)allocator.zero_allocate(size, sizeof(baxter_maintenance_msgs__msg__UpdateSource), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_maintenance_msgs__msg__UpdateSource__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_maintenance_msgs__msg__UpdateSource__fini(&data[i - 1]);
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
baxter_maintenance_msgs__msg__UpdateSource__Sequence__fini(baxter_maintenance_msgs__msg__UpdateSource__Sequence * array)
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
      baxter_maintenance_msgs__msg__UpdateSource__fini(&array->data[i]);
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

baxter_maintenance_msgs__msg__UpdateSource__Sequence *
baxter_maintenance_msgs__msg__UpdateSource__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__UpdateSource__Sequence * array = (baxter_maintenance_msgs__msg__UpdateSource__Sequence *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__UpdateSource__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_maintenance_msgs__msg__UpdateSource__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_maintenance_msgs__msg__UpdateSource__Sequence__destroy(baxter_maintenance_msgs__msg__UpdateSource__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_maintenance_msgs__msg__UpdateSource__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_maintenance_msgs__msg__UpdateSource__Sequence__are_equal(const baxter_maintenance_msgs__msg__UpdateSource__Sequence * lhs, const baxter_maintenance_msgs__msg__UpdateSource__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_maintenance_msgs__msg__UpdateSource__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__UpdateSource__Sequence__copy(
  const baxter_maintenance_msgs__msg__UpdateSource__Sequence * input,
  baxter_maintenance_msgs__msg__UpdateSource__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_maintenance_msgs__msg__UpdateSource);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_maintenance_msgs__msg__UpdateSource * data =
      (baxter_maintenance_msgs__msg__UpdateSource *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_maintenance_msgs__msg__UpdateSource__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_maintenance_msgs__msg__UpdateSource__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_maintenance_msgs__msg__UpdateSource__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
