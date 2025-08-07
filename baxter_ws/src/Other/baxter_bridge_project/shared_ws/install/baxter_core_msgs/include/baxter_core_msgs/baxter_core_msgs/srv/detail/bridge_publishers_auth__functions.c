// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:srv/BridgePublishersAuth.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/srv/detail/bridge_publishers_auth__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `topic`
// Member `user`
#include "rosidl_runtime_c/string_functions.h"

bool
baxter_core_msgs__srv__BridgePublishersAuth_Request__init(baxter_core_msgs__srv__BridgePublishersAuth_Request * msg)
{
  if (!msg) {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__init(&msg->topic)) {
    baxter_core_msgs__srv__BridgePublishersAuth_Request__fini(msg);
    return false;
  }
  // user
  if (!rosidl_runtime_c__String__init(&msg->user)) {
    baxter_core_msgs__srv__BridgePublishersAuth_Request__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__srv__BridgePublishersAuth_Request__fini(baxter_core_msgs__srv__BridgePublishersAuth_Request * msg)
{
  if (!msg) {
    return;
  }
  // topic
  rosidl_runtime_c__String__fini(&msg->topic);
  // user
  rosidl_runtime_c__String__fini(&msg->user);
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Request__are_equal(const baxter_core_msgs__srv__BridgePublishersAuth_Request * lhs, const baxter_core_msgs__srv__BridgePublishersAuth_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->topic), &(rhs->topic)))
  {
    return false;
  }
  // user
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->user), &(rhs->user)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Request__copy(
  const baxter_core_msgs__srv__BridgePublishersAuth_Request * input,
  baxter_core_msgs__srv__BridgePublishersAuth_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // topic
  if (!rosidl_runtime_c__String__copy(
      &(input->topic), &(output->topic)))
  {
    return false;
  }
  // user
  if (!rosidl_runtime_c__String__copy(
      &(input->user), &(output->user)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__srv__BridgePublishersAuth_Request *
baxter_core_msgs__srv__BridgePublishersAuth_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__BridgePublishersAuth_Request * msg = (baxter_core_msgs__srv__BridgePublishersAuth_Request *)allocator.allocate(sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Request));
  bool success = baxter_core_msgs__srv__BridgePublishersAuth_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__srv__BridgePublishersAuth_Request__destroy(baxter_core_msgs__srv__BridgePublishersAuth_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__srv__BridgePublishersAuth_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__init(baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__BridgePublishersAuth_Request * data = NULL;

  if (size) {
    data = (baxter_core_msgs__srv__BridgePublishersAuth_Request *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__srv__BridgePublishersAuth_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__srv__BridgePublishersAuth_Request__fini(&data[i - 1]);
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
baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__fini(baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * array)
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
      baxter_core_msgs__srv__BridgePublishersAuth_Request__fini(&array->data[i]);
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

baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence *
baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * array = (baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__destroy(baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__are_equal(const baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * lhs, const baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__srv__BridgePublishersAuth_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence__copy(
  const baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * input,
  baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__srv__BridgePublishersAuth_Request * data =
      (baxter_core_msgs__srv__BridgePublishersAuth_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__srv__BridgePublishersAuth_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__srv__BridgePublishersAuth_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__srv__BridgePublishersAuth_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `publishers`
#include "baxter_core_msgs/msg/detail/bridge_publisher__functions.h"
// Member `forced_left`
// Member `forced_right`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
baxter_core_msgs__srv__BridgePublishersAuth_Response__init(baxter_core_msgs__srv__BridgePublishersAuth_Response * msg)
{
  if (!msg) {
    return false;
  }
  // publishers
  if (!baxter_core_msgs__msg__BridgePublisher__Sequence__init(&msg->publishers, 0)) {
    baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(msg);
    return false;
  }
  // forced_left
  if (!rosidl_runtime_c__String__init(&msg->forced_left)) {
    baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(msg);
    return false;
  }
  // forced_right
  if (!rosidl_runtime_c__String__init(&msg->forced_right)) {
    baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(baxter_core_msgs__srv__BridgePublishersAuth_Response * msg)
{
  if (!msg) {
    return;
  }
  // publishers
  baxter_core_msgs__msg__BridgePublisher__Sequence__fini(&msg->publishers);
  // forced_left
  rosidl_runtime_c__String__fini(&msg->forced_left);
  // forced_right
  rosidl_runtime_c__String__fini(&msg->forced_right);
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Response__are_equal(const baxter_core_msgs__srv__BridgePublishersAuth_Response * lhs, const baxter_core_msgs__srv__BridgePublishersAuth_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // publishers
  if (!baxter_core_msgs__msg__BridgePublisher__Sequence__are_equal(
      &(lhs->publishers), &(rhs->publishers)))
  {
    return false;
  }
  // forced_left
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->forced_left), &(rhs->forced_left)))
  {
    return false;
  }
  // forced_right
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->forced_right), &(rhs->forced_right)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Response__copy(
  const baxter_core_msgs__srv__BridgePublishersAuth_Response * input,
  baxter_core_msgs__srv__BridgePublishersAuth_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // publishers
  if (!baxter_core_msgs__msg__BridgePublisher__Sequence__copy(
      &(input->publishers), &(output->publishers)))
  {
    return false;
  }
  // forced_left
  if (!rosidl_runtime_c__String__copy(
      &(input->forced_left), &(output->forced_left)))
  {
    return false;
  }
  // forced_right
  if (!rosidl_runtime_c__String__copy(
      &(input->forced_right), &(output->forced_right)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__srv__BridgePublishersAuth_Response *
baxter_core_msgs__srv__BridgePublishersAuth_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__BridgePublishersAuth_Response * msg = (baxter_core_msgs__srv__BridgePublishersAuth_Response *)allocator.allocate(sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Response));
  bool success = baxter_core_msgs__srv__BridgePublishersAuth_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__srv__BridgePublishersAuth_Response__destroy(baxter_core_msgs__srv__BridgePublishersAuth_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__init(baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__BridgePublishersAuth_Response * data = NULL;

  if (size) {
    data = (baxter_core_msgs__srv__BridgePublishersAuth_Response *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__srv__BridgePublishersAuth_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(&data[i - 1]);
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
baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__fini(baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * array)
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
      baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(&array->data[i]);
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

baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence *
baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * array = (baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__destroy(baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__are_equal(const baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * lhs, const baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__srv__BridgePublishersAuth_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence__copy(
  const baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * input,
  baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__srv__BridgePublishersAuth_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__srv__BridgePublishersAuth_Response * data =
      (baxter_core_msgs__srv__BridgePublishersAuth_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__srv__BridgePublishersAuth_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__srv__BridgePublishersAuth_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__srv__BridgePublishersAuth_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
