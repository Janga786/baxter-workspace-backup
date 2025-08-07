// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/CollisionAvoidanceState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__STRUCT_H_

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
// Member 'collision_object'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CollisionAvoidanceState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__CollisionAvoidanceState
{
  std_msgs__msg__Header header;
  bool other_arm;
  rosidl_runtime_c__String__Sequence collision_object;
} baxter_core_msgs__msg__CollisionAvoidanceState;

// Struct for a sequence of baxter_core_msgs__msg__CollisionAvoidanceState.
typedef struct baxter_core_msgs__msg__CollisionAvoidanceState__Sequence
{
  baxter_core_msgs__msg__CollisionAvoidanceState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__CollisionAvoidanceState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__STRUCT_H_
