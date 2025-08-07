// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/AnalogIOStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATES__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'names'
#include "rosidl_runtime_c/string.h"
// Member 'states'
#include "baxter_core_msgs/msg/detail/analog_io_state__struct.h"

/// Struct defined in msg/AnalogIOStates in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__AnalogIOStates
{
  rosidl_runtime_c__String__Sequence names;
  baxter_core_msgs__msg__AnalogIOState__Sequence states;
} baxter_core_msgs__msg__AnalogIOStates;

// Struct for a sequence of baxter_core_msgs__msg__AnalogIOStates.
typedef struct baxter_core_msgs__msg__AnalogIOStates__Sequence
{
  baxter_core_msgs__msg__AnalogIOStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__AnalogIOStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_IO_STATES__STRUCT_H_
