// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/CollisionDetectionState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_DETECTION_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_DETECTION_STATE__STRUCT_H_

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

/// Struct defined in msg/CollisionDetectionState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__CollisionDetectionState
{
  std_msgs__msg__Header header;
  bool collision_state;
} baxter_core_msgs__msg__CollisionDetectionState;

// Struct for a sequence of baxter_core_msgs__msg__CollisionDetectionState.
typedef struct baxter_core_msgs__msg__CollisionDetectionState__Sequence
{
  baxter_core_msgs__msg__CollisionDetectionState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__CollisionDetectionState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_DETECTION_STATE__STRUCT_H_
