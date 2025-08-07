// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/AnalogOutputCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/AnalogOutputCommand in the package baxter_core_msgs.
/**
  * the name of the output
 */
typedef struct baxter_core_msgs__msg__AnalogOutputCommand
{
  rosidl_runtime_c__String name;
  /// the value to set output
  uint16_t value;
} baxter_core_msgs__msg__AnalogOutputCommand;

// Struct for a sequence of baxter_core_msgs__msg__AnalogOutputCommand.
typedef struct baxter_core_msgs__msg__AnalogOutputCommand__Sequence
{
  baxter_core_msgs__msg__AnalogOutputCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__AnalogOutputCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__STRUCT_H_
