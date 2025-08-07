// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/NavigatorState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'button_names'
// Member 'light_names'
#include "rosidl_runtime_c/string.h"
// Member 'buttons'
// Member 'lights'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/NavigatorState in the package baxter_core_msgs.
/**
  * buttons
 */
typedef struct baxter_core_msgs__msg__NavigatorState
{
  rosidl_runtime_c__String__Sequence button_names;
  rosidl_runtime_c__boolean__Sequence buttons;
  /// wheel position
  uint8_t wheel;
  /// true if the light is on, false if not
  /// lights map to button names
  rosidl_runtime_c__String__Sequence light_names;
  rosidl_runtime_c__boolean__Sequence lights;
} baxter_core_msgs__msg__NavigatorState;

// Struct for a sequence of baxter_core_msgs__msg__NavigatorState.
typedef struct baxter_core_msgs__msg__NavigatorState__Sequence
{
  baxter_core_msgs__msg__NavigatorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__NavigatorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__STRUCT_H_
