// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:srv/SolvePositionIK.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/srv/detail/solve_position_ik__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pose_stamp`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `seed_angles`
#include "sensor_msgs/msg/detail/joint_state__functions.h"

bool
baxter_core_msgs__srv__SolvePositionIK_Request__init(baxter_core_msgs__srv__SolvePositionIK_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pose_stamp
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->pose_stamp, 0)) {
    baxter_core_msgs__srv__SolvePositionIK_Request__fini(msg);
    return false;
  }
  // seed_angles
  if (!sensor_msgs__msg__JointState__Sequence__init(&msg->seed_angles, 0)) {
    baxter_core_msgs__srv__SolvePositionIK_Request__fini(msg);
    return false;
  }
  // seed_mode
  return true;
}

void
baxter_core_msgs__srv__SolvePositionIK_Request__fini(baxter_core_msgs__srv__SolvePositionIK_Request * msg)
{
  if (!msg) {
    return;
  }
  // pose_stamp
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->pose_stamp);
  // seed_angles
  sensor_msgs__msg__JointState__Sequence__fini(&msg->seed_angles);
  // seed_mode
}

bool
baxter_core_msgs__srv__SolvePositionIK_Request__are_equal(const baxter_core_msgs__srv__SolvePositionIK_Request * lhs, const baxter_core_msgs__srv__SolvePositionIK_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose_stamp
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
      &(lhs->pose_stamp), &(rhs->pose_stamp)))
  {
    return false;
  }
  // seed_angles
  if (!sensor_msgs__msg__JointState__Sequence__are_equal(
      &(lhs->seed_angles), &(rhs->seed_angles)))
  {
    return false;
  }
  // seed_mode
  if (lhs->seed_mode != rhs->seed_mode) {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__srv__SolvePositionIK_Request__copy(
  const baxter_core_msgs__srv__SolvePositionIK_Request * input,
  baxter_core_msgs__srv__SolvePositionIK_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pose_stamp
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
      &(input->pose_stamp), &(output->pose_stamp)))
  {
    return false;
  }
  // seed_angles
  if (!sensor_msgs__msg__JointState__Sequence__copy(
      &(input->seed_angles), &(output->seed_angles)))
  {
    return false;
  }
  // seed_mode
  output->seed_mode = input->seed_mode;
  return true;
}

baxter_core_msgs__srv__SolvePositionIK_Request *
baxter_core_msgs__srv__SolvePositionIK_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__SolvePositionIK_Request * msg = (baxter_core_msgs__srv__SolvePositionIK_Request *)allocator.allocate(sizeof(baxter_core_msgs__srv__SolvePositionIK_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__srv__SolvePositionIK_Request));
  bool success = baxter_core_msgs__srv__SolvePositionIK_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__srv__SolvePositionIK_Request__destroy(baxter_core_msgs__srv__SolvePositionIK_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__srv__SolvePositionIK_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__init(baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__SolvePositionIK_Request * data = NULL;

  if (size) {
    data = (baxter_core_msgs__srv__SolvePositionIK_Request *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__srv__SolvePositionIK_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__srv__SolvePositionIK_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__srv__SolvePositionIK_Request__fini(&data[i - 1]);
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
baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__fini(baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * array)
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
      baxter_core_msgs__srv__SolvePositionIK_Request__fini(&array->data[i]);
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

baxter_core_msgs__srv__SolvePositionIK_Request__Sequence *
baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * array = (baxter_core_msgs__srv__SolvePositionIK_Request__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__srv__SolvePositionIK_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__destroy(baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__are_equal(const baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * lhs, const baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__srv__SolvePositionIK_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__srv__SolvePositionIK_Request__Sequence__copy(
  const baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * input,
  baxter_core_msgs__srv__SolvePositionIK_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__srv__SolvePositionIK_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__srv__SolvePositionIK_Request * data =
      (baxter_core_msgs__srv__SolvePositionIK_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__srv__SolvePositionIK_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__srv__SolvePositionIK_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__srv__SolvePositionIK_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `joints`
// already included above
// #include "sensor_msgs/msg/detail/joint_state__functions.h"
// Member `is_valid`
// Member `result_type`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
baxter_core_msgs__srv__SolvePositionIK_Response__init(baxter_core_msgs__srv__SolvePositionIK_Response * msg)
{
  if (!msg) {
    return false;
  }
  // joints
  if (!sensor_msgs__msg__JointState__Sequence__init(&msg->joints, 0)) {
    baxter_core_msgs__srv__SolvePositionIK_Response__fini(msg);
    return false;
  }
  // is_valid
  if (!rosidl_runtime_c__boolean__Sequence__init(&msg->is_valid, 0)) {
    baxter_core_msgs__srv__SolvePositionIK_Response__fini(msg);
    return false;
  }
  // result_type
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->result_type, 0)) {
    baxter_core_msgs__srv__SolvePositionIK_Response__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__srv__SolvePositionIK_Response__fini(baxter_core_msgs__srv__SolvePositionIK_Response * msg)
{
  if (!msg) {
    return;
  }
  // joints
  sensor_msgs__msg__JointState__Sequence__fini(&msg->joints);
  // is_valid
  rosidl_runtime_c__boolean__Sequence__fini(&msg->is_valid);
  // result_type
  rosidl_runtime_c__uint8__Sequence__fini(&msg->result_type);
}

bool
baxter_core_msgs__srv__SolvePositionIK_Response__are_equal(const baxter_core_msgs__srv__SolvePositionIK_Response * lhs, const baxter_core_msgs__srv__SolvePositionIK_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joints
  if (!sensor_msgs__msg__JointState__Sequence__are_equal(
      &(lhs->joints), &(rhs->joints)))
  {
    return false;
  }
  // is_valid
  if (!rosidl_runtime_c__boolean__Sequence__are_equal(
      &(lhs->is_valid), &(rhs->is_valid)))
  {
    return false;
  }
  // result_type
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->result_type), &(rhs->result_type)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__srv__SolvePositionIK_Response__copy(
  const baxter_core_msgs__srv__SolvePositionIK_Response * input,
  baxter_core_msgs__srv__SolvePositionIK_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // joints
  if (!sensor_msgs__msg__JointState__Sequence__copy(
      &(input->joints), &(output->joints)))
  {
    return false;
  }
  // is_valid
  if (!rosidl_runtime_c__boolean__Sequence__copy(
      &(input->is_valid), &(output->is_valid)))
  {
    return false;
  }
  // result_type
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->result_type), &(output->result_type)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__srv__SolvePositionIK_Response *
baxter_core_msgs__srv__SolvePositionIK_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__SolvePositionIK_Response * msg = (baxter_core_msgs__srv__SolvePositionIK_Response *)allocator.allocate(sizeof(baxter_core_msgs__srv__SolvePositionIK_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__srv__SolvePositionIK_Response));
  bool success = baxter_core_msgs__srv__SolvePositionIK_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__srv__SolvePositionIK_Response__destroy(baxter_core_msgs__srv__SolvePositionIK_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__srv__SolvePositionIK_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__init(baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__SolvePositionIK_Response * data = NULL;

  if (size) {
    data = (baxter_core_msgs__srv__SolvePositionIK_Response *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__srv__SolvePositionIK_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__srv__SolvePositionIK_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__srv__SolvePositionIK_Response__fini(&data[i - 1]);
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
baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__fini(baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * array)
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
      baxter_core_msgs__srv__SolvePositionIK_Response__fini(&array->data[i]);
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

baxter_core_msgs__srv__SolvePositionIK_Response__Sequence *
baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * array = (baxter_core_msgs__srv__SolvePositionIK_Response__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__srv__SolvePositionIK_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__destroy(baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__are_equal(const baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * lhs, const baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__srv__SolvePositionIK_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__srv__SolvePositionIK_Response__Sequence__copy(
  const baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * input,
  baxter_core_msgs__srv__SolvePositionIK_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__srv__SolvePositionIK_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__srv__SolvePositionIK_Response * data =
      (baxter_core_msgs__srv__SolvePositionIK_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__srv__SolvePositionIK_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__srv__SolvePositionIK_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__srv__SolvePositionIK_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
