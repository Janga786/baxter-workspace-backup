// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/SEAJointState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'commanded_position'
// Member 'commanded_velocity'
// Member 'commanded_acceleration'
// Member 'commanded_effort'
// Member 'actual_position'
// Member 'actual_velocity'
// Member 'actual_effort'
// Member 'gravity_model_effort'
// Member 'gravity_only'
// Member 'hysteresis_model_effort'
// Member 'crosstalk_model_effort'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SEAJointState in the package baxter_core_msgs.
/**
  * This is a message that holds data to describe the state of a set of torque controlled joints.
  *
  * The state of each joint (revolute or prismatic) is defined by:
  *  * the position of the joint (rad or m),
  *  * the velocity of the joint (rad/s or m/s) and
  *  * the effort that is applied in the joint (Nm or N).
  *
  * Each joint is uniquely identified by its name
  * The header specifies the time at which the joint states were recorded. All the joint states
  * in one message have to be recorded at the same time.
  *
  * This message consists of a multiple arrays, one for each part of the joint state.
  * The goal is to make each of the fields optional. When e.g. your joints have no
  * effort associated with them, you can leave the effort array empty.
  *
  * All arrays in this message should have the same size, or be empty.
  * This is the only way to uniquely associate the joint name with the correct
  * states.
 */
typedef struct baxter_core_msgs__msg__SEAJointState
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence commanded_position;
  rosidl_runtime_c__double__Sequence commanded_velocity;
  rosidl_runtime_c__double__Sequence commanded_acceleration;
  rosidl_runtime_c__double__Sequence commanded_effort;
  rosidl_runtime_c__double__Sequence actual_position;
  rosidl_runtime_c__double__Sequence actual_velocity;
  rosidl_runtime_c__double__Sequence actual_effort;
  /// This includes the inertial feed forward torques when applicable.
  rosidl_runtime_c__double__Sequence gravity_model_effort;
  /// This is the torque required to hold the arm against gravity returned by KDL
  /// if the arm was stationary.  This does not include inertial feed forward
  /// torques (even when we have them) or any of the corrections (i.e. spring
  /// hysteresis, crosstalk, etc) we make to the KDL model.
  rosidl_runtime_c__double__Sequence gravity_only;
  rosidl_runtime_c__double__Sequence hysteresis_model_effort;
  rosidl_runtime_c__double__Sequence crosstalk_model_effort;
  double hyst_state;
} baxter_core_msgs__msg__SEAJointState;

// Struct for a sequence of baxter_core_msgs__msg__SEAJointState.
typedef struct baxter_core_msgs__msg__SEAJointState__Sequence
{
  baxter_core_msgs__msg__SEAJointState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__SEAJointState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__STRUCT_H_
