// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_maintenance_msgs:msg/TareData.idl
// generated code does not contain a copyright notice
#include "baxter_maintenance_msgs/msg/detail/tare_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
baxter_maintenance_msgs__msg__TareData__init(baxter_maintenance_msgs__msg__TareData * msg)
{
  if (!msg) {
    return false;
  }
  // tune_gravity_spring
  return true;
}

void
baxter_maintenance_msgs__msg__TareData__fini(baxter_maintenance_msgs__msg__TareData * msg)
{
  if (!msg) {
    return;
  }
  // tune_gravity_spring
}

bool
baxter_maintenance_msgs__msg__TareData__are_equal(const baxter_maintenance_msgs__msg__TareData * lhs, const baxter_maintenance_msgs__msg__TareData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // tune_gravity_spring
  if (lhs->tune_gravity_spring != rhs->tune_gravity_spring) {
    return false;
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__TareData__copy(
  const baxter_maintenance_msgs__msg__TareData * input,
  baxter_maintenance_msgs__msg__TareData * output)
{
  if (!input || !output) {
    return false;
  }
  // tune_gravity_spring
  output->tune_gravity_spring = input->tune_gravity_spring;
  return true;
}

baxter_maintenance_msgs__msg__TareData *
baxter_maintenance_msgs__msg__TareData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__TareData * msg = (baxter_maintenance_msgs__msg__TareData *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__TareData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_maintenance_msgs__msg__TareData));
  bool success = baxter_maintenance_msgs__msg__TareData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_maintenance_msgs__msg__TareData__destroy(baxter_maintenance_msgs__msg__TareData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_maintenance_msgs__msg__TareData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_maintenance_msgs__msg__TareData__Sequence__init(baxter_maintenance_msgs__msg__TareData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__TareData * data = NULL;

  if (size) {
    data = (baxter_maintenance_msgs__msg__TareData *)allocator.zero_allocate(size, sizeof(baxter_maintenance_msgs__msg__TareData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_maintenance_msgs__msg__TareData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_maintenance_msgs__msg__TareData__fini(&data[i - 1]);
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
baxter_maintenance_msgs__msg__TareData__Sequence__fini(baxter_maintenance_msgs__msg__TareData__Sequence * array)
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
      baxter_maintenance_msgs__msg__TareData__fini(&array->data[i]);
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

baxter_maintenance_msgs__msg__TareData__Sequence *
baxter_maintenance_msgs__msg__TareData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_maintenance_msgs__msg__TareData__Sequence * array = (baxter_maintenance_msgs__msg__TareData__Sequence *)allocator.allocate(sizeof(baxter_maintenance_msgs__msg__TareData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_maintenance_msgs__msg__TareData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_maintenance_msgs__msg__TareData__Sequence__destroy(baxter_maintenance_msgs__msg__TareData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_maintenance_msgs__msg__TareData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_maintenance_msgs__msg__TareData__Sequence__are_equal(const baxter_maintenance_msgs__msg__TareData__Sequence * lhs, const baxter_maintenance_msgs__msg__TareData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_maintenance_msgs__msg__TareData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_maintenance_msgs__msg__TareData__Sequence__copy(
  const baxter_maintenance_msgs__msg__TareData__Sequence * input,
  baxter_maintenance_msgs__msg__TareData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_maintenance_msgs__msg__TareData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_maintenance_msgs__msg__TareData * data =
      (baxter_maintenance_msgs__msg__TareData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_maintenance_msgs__msg__TareData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_maintenance_msgs__msg__TareData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_maintenance_msgs__msg__TareData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
