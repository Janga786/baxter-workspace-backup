// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_maintenance_msgs:msg/TareEnable.idl
// generated code does not contain a copyright notice
#include "baxter_maintenance_msgs/msg/detail/tare_enable__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `uid`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "baxter_maintenance_msgs/msg/detail/tare_data__functions.h"

bool
baxter_maintenance_msgs__msg__TareEnable__init(baxter_maintenance_msgs__msg__TareEnable * msg)
{
  if (!msg) {
    return false;
  }
  // is_enabled
  // uid
  if (!rosidl_runtime_c__String__init(&msg->uid)) {
    baxter_maintenance_msgs__msg__TareEnable__fini(msg);
    return false;
  }
  // data
  if (!baxter_maintenance_msgs__msg__TareData__init(&msg->data)) {
    baxter_maintenance_msgs__msg__TareEnable__fini(msg);
    return false;
  }
  return true;
}

void
baxter_maintenance_msgs__msg__TareEnable__fini(baxter_maintenance_msgs__msg__TareEnable * msg)
{
  if (!msg) {
    return;
  }
  // is_enabled
  // uid
  rosidl_runtime_c__String__fini(&msg->uid);
  // data
  baxter_maintenance_msgs__msg__TareData__fini(&msg->data);
}

bool
baxter_maintenance_msgs__msg__TareEnable__are_equal(const baxter_maintenance_msgs__msg__TareEnable * lhs, const baxter_maintenance_msgs__msg__TareEnable * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // is_enabled
  if (lhs->is_enabled != rhs->is_enabled) {
    return false;
  }
  // uid
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->uid), &(rhs->uid)))
  {
    return false;
  }
  // data
  if (!baxter_maintenance_msgs__msg__TareData__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__TareEnable__copy(
  const baxter_maintenance_msgs__msg__TareEnable * input,
  baxter_maintenance_msgs__msg__TareEnable * output)
{
  if (!input || !output) {
    return false;
  }
  // is_enabled
  output->is_enabled = input->is_enabled;
  // uid
  if (!rosidl_runtime_c__String__copy(
      &(input->uid), &(output->uid)))
  {
    return false;
  }
  // data
  if (!baxter_maintenance_msgs__msg__TareData__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

baxter_maintenance_msgs__msg__TareEnable *
baxter_maintenance_msgs__msg__TareEnable__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__TareEnable * msg = (baxter_maintenance_msgs__msg__TareEnable *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__TareEnable), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_maintenance_msgs__msg__TareEnable));
  bool success = baxter_maintenance_msgs__msg__TareEnable__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_maintenance_msgs__msg__TareEnable__destroy(baxter_maintenance_msgs__msg__TareEnable * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_maintenance_msgs__msg__TareEnable__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_maintenance_msgs__msg__TareEnable__Sequence__init(baxter_maintenance_msgs__msg__TareEnable__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__TareEnable * data = NULL;

  if (size) {
    data = (baxter_maintenance_msgs__msg__TareEnable *)allocator.zero_allocate(size, sizeof(baxter_maintenance_msgs__msg__TareEnable), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_maintenance_msgs__msg__TareEnable__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_maintenance_msgs__msg__TareEnable__fini(&data[i - 1]);
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
baxter_maintenance_msgs__msg__TareEnable__Sequence__fini(baxter_maintenance_msgs__msg__TareEnable__Sequence * array)
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
      baxter_maintenance_msgs__msg__TareEnable__fini(&array->data[i]);
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

baxter_maintenance_msgs__msg__TareEnable__Sequence *
baxter_maintenance_msgs__msg__TareEnable__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__TareEnable__Sequence * array = (baxter_maintenance_msgs__msg__TareEnable__Sequence *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__TareEnable__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_maintenance_msgs__msg__TareEnable__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_maintenance_msgs__msg__TareEnable__Sequence__destroy(baxter_maintenance_msgs__msg__TareEnable__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_maintenance_msgs__msg__TareEnable__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_maintenance_msgs__msg__TareEnable__Sequence__are_equal(const baxter_maintenance_msgs__msg__TareEnable__Sequence * lhs, const baxter_maintenance_msgs__msg__TareEnable__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_maintenance_msgs__msg__TareEnable__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__TareEnable__Sequence__copy(
  const baxter_maintenance_msgs__msg__TareEnable__Sequence * input,
  baxter_maintenance_msgs__msg__TareEnable__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_maintenance_msgs__msg__TareEnable);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_maintenance_msgs__msg__TareEnable * data =
      (baxter_maintenance_msgs__msg__TareEnable *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_maintenance_msgs__msg__TareEnable__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_maintenance_msgs__msg__TareEnable__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_maintenance_msgs__msg__TareEnable__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
