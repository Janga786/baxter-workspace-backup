// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/camera_settings__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `controls`
#include "baxter_core_msgs/msg/detail/camera_control__functions.h"

bool
baxter_core_msgs__msg__CameraSettings__init(baxter_core_msgs__msg__CameraSettings * msg)
{
  if (!msg) {
    return false;
  }
  // width
  // height
  // fps
  // controls
  if (!baxter_core_msgs__msg__CameraControl__Sequence__init(&msg->controls, 0)) {
    baxter_core_msgs__msg__CameraSettings__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__msg__CameraSettings__fini(baxter_core_msgs__msg__CameraSettings * msg)
{
  if (!msg) {
    return;
  }
  // width
  // height
  // fps
  // controls
  baxter_core_msgs__msg__CameraControl__Sequence__fini(&msg->controls);
}

bool
baxter_core_msgs__msg__CameraSettings__are_equal(const baxter_core_msgs__msg__CameraSettings * lhs, const baxter_core_msgs__msg__CameraSettings * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // fps
  if (lhs->fps != rhs->fps) {
    return false;
  }
  // controls
  if (!baxter_core_msgs__msg__CameraControl__Sequence__are_equal(
      &(lhs->controls), &(rhs->controls)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__CameraSettings__copy(
  const baxter_core_msgs__msg__CameraSettings * input,
  baxter_core_msgs__msg__CameraSettings * output)
{
  if (!input || !output) {
    return false;
  }
  // width
  output->width = input->width;
  // height
  output->height = input->height;
  // fps
  output->fps = input->fps;
  // controls
  if (!baxter_core_msgs__msg__CameraControl__Sequence__copy(
      &(input->controls), &(output->controls)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__msg__CameraSettings *
baxter_core_msgs__msg__CameraSettings__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__CameraSettings * msg = (baxter_core_msgs__msg__CameraSettings *)allocator.allocate(sizeof(baxter_core_msgs__msg__CameraSettings), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__CameraSettings));
  bool success = baxter_core_msgs__msg__CameraSettings__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__CameraSettings__destroy(baxter_core_msgs__msg__CameraSettings * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__CameraSettings__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__CameraSettings__Sequence__init(baxter_core_msgs__msg__CameraSettings__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__CameraSettings * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__CameraSettings *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__CameraSettings), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__CameraSettings__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__CameraSettings__fini(&data[i - 1]);
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
baxter_core_msgs__msg__CameraSettings__Sequence__fini(baxter_core_msgs__msg__CameraSettings__Sequence * array)
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
      baxter_core_msgs__msg__CameraSettings__fini(&array->data[i]);
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

baxter_core_msgs__msg__CameraSettings__Sequence *
baxter_core_msgs__msg__CameraSettings__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__CameraSettings__Sequence * array = (baxter_core_msgs__msg__CameraSettings__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__CameraSettings__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__CameraSettings__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__CameraSettings__Sequence__destroy(baxter_core_msgs__msg__CameraSettings__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__CameraSettings__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__CameraSettings__Sequence__are_equal(const baxter_core_msgs__msg__CameraSettings__Sequence * lhs, const baxter_core_msgs__msg__CameraSettings__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__CameraSettings__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__CameraSettings__Sequence__copy(
  const baxter_core_msgs__msg__CameraSettings__Sequence * input,
  baxter_core_msgs__msg__CameraSettings__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__CameraSettings);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__CameraSettings * data =
      (baxter_core_msgs__msg__CameraSettings *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__CameraSettings__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__CameraSettings__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__CameraSettings__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
