// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/SEAJointState.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/sea_joint_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `commanded_position`
// Member `commanded_velocity`
// Member `commanded_acceleration`
// Member `commanded_effort`
// Member `actual_position`
// Member `actual_velocity`
// Member `actual_effort`
// Member `gravity_model_effort`
// Member `gravity_only`
// Member `hysteresis_model_effort`
// Member `crosstalk_model_effort`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
baxter_core_msgs__msg__SEAJointState__init(baxter_core_msgs__msg__SEAJointState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__Sequence__init(&msg->name, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // commanded_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->commanded_position, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // commanded_velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->commanded_velocity, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // commanded_acceleration
  if (!rosidl_runtime_c__double__Sequence__init(&msg->commanded_acceleration, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // commanded_effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->commanded_effort, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // actual_position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->actual_position, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // actual_velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->actual_velocity, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // actual_effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->actual_effort, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // gravity_model_effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->gravity_model_effort, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // gravity_only
  if (!rosidl_runtime_c__double__Sequence__init(&msg->gravity_only, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // hysteresis_model_effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->hysteresis_model_effort, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // crosstalk_model_effort
  if (!rosidl_runtime_c__double__Sequence__init(&msg->crosstalk_model_effort, 0)) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
    return false;
  }
  // hyst_state
  return true;
}

void
baxter_core_msgs__msg__SEAJointState__fini(baxter_core_msgs__msg__SEAJointState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // name
  rosidl_runtime_c__String__Sequence__fini(&msg->name);
  // commanded_position
  rosidl_runtime_c__double__Sequence__fini(&msg->commanded_position);
  // commanded_velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->commanded_velocity);
  // commanded_acceleration
  rosidl_runtime_c__double__Sequence__fini(&msg->commanded_acceleration);
  // commanded_effort
  rosidl_runtime_c__double__Sequence__fini(&msg->commanded_effort);
  // actual_position
  rosidl_runtime_c__double__Sequence__fini(&msg->actual_position);
  // actual_velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->actual_velocity);
  // actual_effort
  rosidl_runtime_c__double__Sequence__fini(&msg->actual_effort);
  // gravity_model_effort
  rosidl_runtime_c__double__Sequence__fini(&msg->gravity_model_effort);
  // gravity_only
  rosidl_runtime_c__double__Sequence__fini(&msg->gravity_only);
  // hysteresis_model_effort
  rosidl_runtime_c__double__Sequence__fini(&msg->hysteresis_model_effort);
  // crosstalk_model_effort
  rosidl_runtime_c__double__Sequence__fini(&msg->crosstalk_model_effort);
  // hyst_state
}

bool
baxter_core_msgs__msg__SEAJointState__are_equal(const baxter_core_msgs__msg__SEAJointState * lhs, const baxter_core_msgs__msg__SEAJointState * rhs)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // commanded_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->commanded_position), &(rhs->commanded_position)))
  {
    return false;
  }
  // commanded_velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->commanded_velocity), &(rhs->commanded_velocity)))
  {
    return false;
  }
  // commanded_acceleration
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->commanded_acceleration), &(rhs->commanded_acceleration)))
  {
    return false;
  }
  // commanded_effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->commanded_effort), &(rhs->commanded_effort)))
  {
    return false;
  }
  // actual_position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->actual_position), &(rhs->actual_position)))
  {
    return false;
  }
  // actual_velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->actual_velocity), &(rhs->actual_velocity)))
  {
    return false;
  }
  // actual_effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->actual_effort), &(rhs->actual_effort)))
  {
    return false;
  }
  // gravity_model_effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->gravity_model_effort), &(rhs->gravity_model_effort)))
  {
    return false;
  }
  // gravity_only
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->gravity_only), &(rhs->gravity_only)))
  {
    return false;
  }
  // hysteresis_model_effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->hysteresis_model_effort), &(rhs->hysteresis_model_effort)))
  {
    return false;
  }
  // crosstalk_model_effort
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->crosstalk_model_effort), &(rhs->crosstalk_model_effort)))
  {
    return false;
  }
  // hyst_state
  if (lhs->hyst_state != rhs->hyst_state) {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__SEAJointState__copy(
  const baxter_core_msgs__msg__SEAJointState * input,
  baxter_core_msgs__msg__SEAJointState * output)
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
  // name
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // commanded_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->commanded_position), &(output->commanded_position)))
  {
    return false;
  }
  // commanded_velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->commanded_velocity), &(output->commanded_velocity)))
  {
    return false;
  }
  // commanded_acceleration
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->commanded_acceleration), &(output->commanded_acceleration)))
  {
    return false;
  }
  // commanded_effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->commanded_effort), &(output->commanded_effort)))
  {
    return false;
  }
  // actual_position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->actual_position), &(output->actual_position)))
  {
    return false;
  }
  // actual_velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->actual_velocity), &(output->actual_velocity)))
  {
    return false;
  }
  // actual_effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->actual_effort), &(output->actual_effort)))
  {
    return false;
  }
  // gravity_model_effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->gravity_model_effort), &(output->gravity_model_effort)))
  {
    return false;
  }
  // gravity_only
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->gravity_only), &(output->gravity_only)))
  {
    return false;
  }
  // hysteresis_model_effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->hysteresis_model_effort), &(output->hysteresis_model_effort)))
  {
    return false;
  }
  // crosstalk_model_effort
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->crosstalk_model_effort), &(output->crosstalk_model_effort)))
  {
    return false;
  }
  // hyst_state
  output->hyst_state = input->hyst_state;
  return true;
}

baxter_core_msgs__msg__SEAJointState *
baxter_core_msgs__msg__SEAJointState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__SEAJointState * msg = (baxter_core_msgs__msg__SEAJointState *)allocator.allocate(sizeof(baxter_core_msgs__msg__SEAJointState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__SEAJointState));
  bool success = baxter_core_msgs__msg__SEAJointState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__SEAJointState__destroy(baxter_core_msgs__msg__SEAJointState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__SEAJointState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__SEAJointState__Sequence__init(baxter_core_msgs__msg__SEAJointState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__SEAJointState * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__SEAJointState *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__SEAJointState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__SEAJointState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__SEAJointState__fini(&data[i - 1]);
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
baxter_core_msgs__msg__SEAJointState__Sequence__fini(baxter_core_msgs__msg__SEAJointState__Sequence * array)
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
      baxter_core_msgs__msg__SEAJointState__fini(&array->data[i]);
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

baxter_core_msgs__msg__SEAJointState__Sequence *
baxter_core_msgs__msg__SEAJointState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__SEAJointState__Sequence * array = (baxter_core_msgs__msg__SEAJointState__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__SEAJointState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__SEAJointState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__SEAJointState__Sequence__destroy(baxter_core_msgs__msg__SEAJointState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__SEAJointState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__SEAJointState__Sequence__are_equal(const baxter_core_msgs__msg__SEAJointState__Sequence * lhs, const baxter_core_msgs__msg__SEAJointState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__SEAJointState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__SEAJointState__Sequence__copy(
  const baxter_core_msgs__msg__SEAJointState__Sequence * input,
  baxter_core_msgs__msg__SEAJointState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__SEAJointState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__SEAJointState * data =
      (baxter_core_msgs__msg__SEAJointState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__SEAJointState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__SEAJointState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__SEAJointState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
