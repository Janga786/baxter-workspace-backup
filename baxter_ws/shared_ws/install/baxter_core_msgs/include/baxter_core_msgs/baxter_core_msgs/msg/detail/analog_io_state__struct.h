// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/AnalogIOState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/AnalogIOState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__AnalogIOState
{
  builtin_interfaces__msg__Time timestamp;
  double value;
  bool is_input_only;
} baxter_core_msgs__msg__AnalogIOState;

// Struct for a sequence of baxter_core_msgs__msg__AnalogIOState.
typedef struct baxter_core_msgs__msg__AnalogIOState__Sequence
{
  baxter_core_msgs__msg__AnalogIOState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__AnalogIOState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATE__STRUCT_H_
