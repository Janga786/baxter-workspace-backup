// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from baxter_core_msgs:msg/EndEffectorProperties.idl
// generated code does not contain a copyright notice
#include "baxter_core_msgs/msg/detail/end_effector_properties__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `manufacturer`
// Member `product`
// Member `serial_number`
// Member `hardware_rev`
// Member `firmware_rev`
// Member `firmware_date`
// Member `properties`
#include "rosidl_runtime_c/string_functions.h"

bool
baxter_core_msgs__msg__EndEffectorProperties__init(baxter_core_msgs__msg__EndEffectorProperties * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // ui_type
  // manufacturer
  if (!rosidl_runtime_c__String__init(&msg->manufacturer)) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
    return false;
  }
  // product
  if (!rosidl_runtime_c__String__init(&msg->product)) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__init(&msg->serial_number)) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
    return false;
  }
  // hardware_rev
  if (!rosidl_runtime_c__String__init(&msg->hardware_rev)) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
    return false;
  }
  // firmware_rev
  if (!rosidl_runtime_c__String__init(&msg->firmware_rev)) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
    return false;
  }
  // firmware_date
  if (!rosidl_runtime_c__String__init(&msg->firmware_date)) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
    return false;
  }
  // has_calibration
  // controls_grip
  // senses_grip
  // reverses_grip
  // controls_force
  // senses_force
  // controls_position
  // senses_position
  // properties
  if (!rosidl_runtime_c__String__init(&msg->properties)) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
    return false;
  }
  return true;
}

void
baxter_core_msgs__msg__EndEffectorProperties__fini(baxter_core_msgs__msg__EndEffectorProperties * msg)
{
  if (!msg) {
    return;
  }
  // id
  // ui_type
  // manufacturer
  rosidl_runtime_c__String__fini(&msg->manufacturer);
  // product
  rosidl_runtime_c__String__fini(&msg->product);
  // serial_number
  rosidl_runtime_c__String__fini(&msg->serial_number);
  // hardware_rev
  rosidl_runtime_c__String__fini(&msg->hardware_rev);
  // firmware_rev
  rosidl_runtime_c__String__fini(&msg->firmware_rev);
  // firmware_date
  rosidl_runtime_c__String__fini(&msg->firmware_date);
  // has_calibration
  // controls_grip
  // senses_grip
  // reverses_grip
  // controls_force
  // senses_force
  // controls_position
  // senses_position
  // properties
  rosidl_runtime_c__String__fini(&msg->properties);
}

bool
baxter_core_msgs__msg__EndEffectorProperties__are_equal(const baxter_core_msgs__msg__EndEffectorProperties * lhs, const baxter_core_msgs__msg__EndEffectorProperties * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // ui_type
  if (lhs->ui_type != rhs->ui_type) {
    return false;
  }
  // manufacturer
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->manufacturer), &(rhs->manufacturer)))
  {
    return false;
  }
  // product
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->product), &(rhs->product)))
  {
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->serial_number), &(rhs->serial_number)))
  {
    return false;
  }
  // hardware_rev
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->hardware_rev), &(rhs->hardware_rev)))
  {
    return false;
  }
  // firmware_rev
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->firmware_rev), &(rhs->firmware_rev)))
  {
    return false;
  }
  // firmware_date
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->firmware_date), &(rhs->firmware_date)))
  {
    return false;
  }
  // has_calibration
  if (lhs->has_calibration != rhs->has_calibration) {
    return false;
  }
  // controls_grip
  if (lhs->controls_grip != rhs->controls_grip) {
    return false;
  }
  // senses_grip
  if (lhs->senses_grip != rhs->senses_grip) {
    return false;
  }
  // reverses_grip
  if (lhs->reverses_grip != rhs->reverses_grip) {
    return false;
  }
  // controls_force
  if (lhs->controls_force != rhs->controls_force) {
    return false;
  }
  // senses_force
  if (lhs->senses_force != rhs->senses_force) {
    return false;
  }
  // controls_position
  if (lhs->controls_position != rhs->controls_position) {
    return false;
  }
  // senses_position
  if (lhs->senses_position != rhs->senses_position) {
    return false;
  }
  // properties
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->properties), &(rhs->properties)))
  {
    return false;
  }
  return true;
}

bool
baxter_core_msgs__msg__EndEffectorProperties__copy(
  const baxter_core_msgs__msg__EndEffectorProperties * input,
  baxter_core_msgs__msg__EndEffectorProperties * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // ui_type
  output->ui_type = input->ui_type;
  // manufacturer
  if (!rosidl_runtime_c__String__copy(
      &(input->manufacturer), &(output->manufacturer)))
  {
    return false;
  }
  // product
  if (!rosidl_runtime_c__String__copy(
      &(input->product), &(output->product)))
  {
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__copy(
      &(input->serial_number), &(output->serial_number)))
  {
    return false;
  }
  // hardware_rev
  if (!rosidl_runtime_c__String__copy(
      &(input->hardware_rev), &(output->hardware_rev)))
  {
    return false;
  }
  // firmware_rev
  if (!rosidl_runtime_c__String__copy(
      &(input->firmware_rev), &(output->firmware_rev)))
  {
    return false;
  }
  // firmware_date
  if (!rosidl_runtime_c__String__copy(
      &(input->firmware_date), &(output->firmware_date)))
  {
    return false;
  }
  // has_calibration
  output->has_calibration = input->has_calibration;
  // controls_grip
  output->controls_grip = input->controls_grip;
  // senses_grip
  output->senses_grip = input->senses_grip;
  // reverses_grip
  output->reverses_grip = input->reverses_grip;
  // controls_force
  output->controls_force = input->controls_force;
  // senses_force
  output->senses_force = input->senses_force;
  // controls_position
  output->controls_position = input->controls_position;
  // senses_position
  output->senses_position = input->senses_position;
  // properties
  if (!rosidl_runtime_c__String__copy(
      &(input->properties), &(output->properties)))
  {
    return false;
  }
  return true;
}

baxter_core_msgs__msg__EndEffectorProperties *
baxter_core_msgs__msg__EndEffectorProperties__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndEffectorProperties * msg = (baxter_core_msgs__msg__EndEffectorProperties *)allocator.allocate(sizeof(baxter_core_msgs__msg__EndEffectorProperties), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(baxter_core_msgs__msg__EndEffectorProperties));
  bool success = baxter_core_msgs__msg__EndEffectorProperties__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
baxter_core_msgs__msg__EndEffectorProperties__destroy(baxter_core_msgs__msg__EndEffectorProperties * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    baxter_core_msgs__msg__EndEffectorProperties__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
baxter_core_msgs__msg__EndEffectorProperties__Sequence__init(baxter_core_msgs__msg__EndEffectorProperties__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndEffectorProperties * data = NULL;

  if (size) {
    data = (baxter_core_msgs__msg__EndEffectorProperties *)allocator.zero_allocate(size, sizeof(baxter_core_msgs__msg__EndEffectorProperties), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = baxter_core_msgs__msg__EndEffectorProperties__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        baxter_core_msgs__msg__EndEffectorProperties__fini(&data[i - 1]);
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
baxter_core_msgs__msg__EndEffectorProperties__Sequence__fini(baxter_core_msgs__msg__EndEffectorProperties__Sequence * array)
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
      baxter_core_msgs__msg__EndEffectorProperties__fini(&array->data[i]);
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

baxter_core_msgs__msg__EndEffectorProperties__Sequence *
baxter_core_msgs__msg__EndEffectorProperties__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  baxter_core_msgs__msg__EndEffectorProperties__Sequence * array = (baxter_core_msgs__msg__EndEffectorProperties__Sequence *)allocator.allocate(sizeof(baxter_core_msgs__msg__EndEffectorProperties__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = baxter_core_msgs__msg__EndEffectorProperties__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
baxter_core_msgs__msg__EndEffectorProperties__Sequence__destroy(baxter_core_msgs__msg__EndEffectorProperties__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    baxter_core_msgs__msg__EndEffectorProperties__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
baxter_core_msgs__msg__EndEffectorProperties__Sequence__are_equal(const baxter_core_msgs__msg__EndEffectorProperties__Sequence * lhs, const baxter_core_msgs__msg__EndEffectorProperties__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!baxter_core_msgs__msg__EndEffectorProperties__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
baxter_core_msgs__msg__EndEffectorProperties__Sequence__copy(
  const baxter_core_msgs__msg__EndEffectorProperties__Sequence * input,
  baxter_core_msgs__msg__EndEffectorProperties__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(baxter_core_msgs__msg__EndEffectorProperties);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    baxter_core_msgs__msg__EndEffectorProperties * data =
      (baxter_core_msgs__msg__EndEffectorProperties *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!baxter_core_msgs__msg__EndEffectorProperties__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          baxter_core_msgs__msg__EndEffectorProperties__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!baxter_core_msgs__msg__EndEffectorProperties__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
