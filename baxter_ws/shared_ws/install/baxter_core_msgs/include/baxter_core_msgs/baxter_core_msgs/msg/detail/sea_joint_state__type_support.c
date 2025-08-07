// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_core_msgs:msg/SEAJointState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_core_msgs/msg/detail/sea_joint_state__rosidl_typesupport_introspection_c.h"
#include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_core_msgs/msg/detail/sea_joint_state__functions.h"
#include "baxter_core_msgs/msg/detail/sea_joint_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
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

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_core_msgs__msg__SEAJointState__init(message_memory);
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_fini_function(void * message_memory)
{
  baxter_core_msgs__msg__SEAJointState__fini(message_memory);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__name(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__name(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__name(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__name(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__name(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__name(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_acceleration(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_acceleration(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_acceleration(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_acceleration(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_acceleration(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_acceleration(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__actual_position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__actual_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__actual_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__actual_position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__actual_velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__actual_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__actual_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__actual_velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__actual_effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__actual_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__actual_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__actual_effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__gravity_model_effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__gravity_model_effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__gravity_model_effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__gravity_model_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__gravity_model_effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__gravity_model_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__gravity_model_effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__gravity_model_effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__gravity_only(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__gravity_only(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__gravity_only(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__gravity_only(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__gravity_only(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__gravity_only(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__gravity_only(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__gravity_only(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__hysteresis_model_effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__hysteresis_model_effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__hysteresis_model_effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__hysteresis_model_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__hysteresis_model_effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__hysteresis_model_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__hysteresis_model_effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__hysteresis_model_effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__crosstalk_model_effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__crosstalk_model_effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__crosstalk_model_effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__crosstalk_model_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__crosstalk_model_effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__crosstalk_model_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__crosstalk_model_effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__crosstalk_model_effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_member_array[14] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, name),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__name,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__name,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__name,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__name,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__name,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__name  // resize(index) function pointer
  },
  {
    "commanded_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, commanded_position),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_position,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_position,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_position,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_position,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_position,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_position  // resize(index) function pointer
  },
  {
    "commanded_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, commanded_velocity),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_velocity,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_velocity,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_velocity,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_velocity,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_velocity,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_velocity  // resize(index) function pointer
  },
  {
    "commanded_acceleration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, commanded_acceleration),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_acceleration,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_acceleration,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_acceleration,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_acceleration,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_acceleration,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_acceleration  // resize(index) function pointer
  },
  {
    "commanded_effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, commanded_effort),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__commanded_effort,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__commanded_effort,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__commanded_effort,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__commanded_effort,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__commanded_effort,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__commanded_effort  // resize(index) function pointer
  },
  {
    "actual_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, actual_position),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__actual_position,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_position,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_position,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__actual_position,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__actual_position,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__actual_position  // resize(index) function pointer
  },
  {
    "actual_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, actual_velocity),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__actual_velocity,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_velocity,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_velocity,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__actual_velocity,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__actual_velocity,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__actual_velocity  // resize(index) function pointer
  },
  {
    "actual_effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, actual_effort),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__actual_effort,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__actual_effort,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__actual_effort,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__actual_effort,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__actual_effort,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__actual_effort  // resize(index) function pointer
  },
  {
    "gravity_model_effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, gravity_model_effort),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__gravity_model_effort,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__gravity_model_effort,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__gravity_model_effort,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__gravity_model_effort,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__gravity_model_effort,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__gravity_model_effort  // resize(index) function pointer
  },
  {
    "gravity_only",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, gravity_only),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__gravity_only,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__gravity_only,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__gravity_only,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__gravity_only,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__gravity_only,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__gravity_only  // resize(index) function pointer
  },
  {
    "hysteresis_model_effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, hysteresis_model_effort),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__hysteresis_model_effort,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__hysteresis_model_effort,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__hysteresis_model_effort,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__hysteresis_model_effort,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__hysteresis_model_effort,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__hysteresis_model_effort  // resize(index) function pointer
  },
  {
    "crosstalk_model_effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, crosstalk_model_effort),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__size_function__SEAJointState__crosstalk_model_effort,  // size() function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_const_function__SEAJointState__crosstalk_model_effort,  // get_const(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__get_function__SEAJointState__crosstalk_model_effort,  // get(index) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__fetch_function__SEAJointState__crosstalk_model_effort,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__assign_function__SEAJointState__crosstalk_model_effort,  // assign(index, value) function pointer
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__resize_function__SEAJointState__crosstalk_model_effort  // resize(index) function pointer
  },
  {
    "hyst_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__SEAJointState, hyst_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_members = {
  "baxter_core_msgs__msg",  // message namespace
  "SEAJointState",  // message name
  14,  // number of fields
  sizeof(baxter_core_msgs__msg__SEAJointState),
  baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_member_array,  // message members
  baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_type_support_handle = {
  0,
  &baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, msg, SEAJointState)() {
  baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_core_msgs__msg__SEAJointState__rosidl_typesupport_introspection_c__SEAJointState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
