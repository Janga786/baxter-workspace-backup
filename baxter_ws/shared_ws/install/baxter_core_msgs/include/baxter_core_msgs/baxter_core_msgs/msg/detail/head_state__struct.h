// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/HeadState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/HeadState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__HeadState
{
  float pan;
  bool is_turning;
  bool is_nodding;
  bool is_pan_enabled;
} baxter_core_msgs__msg__HeadState;

// Struct for a sequence of baxter_core_msgs__msg__HeadState.
typedef struct baxter_core_msgs__msg__HeadState__Sequence
{
  baxter_core_msgs__msg__HeadState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__HeadState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__STRUCT_H_
