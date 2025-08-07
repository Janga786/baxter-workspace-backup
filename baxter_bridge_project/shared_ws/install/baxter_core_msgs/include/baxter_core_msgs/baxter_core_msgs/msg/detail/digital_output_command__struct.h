// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/DigitalOutputCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_OUTPUT_COMMAND__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_OUTPUT_COMMAND__STRUCT_H_

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

/// Struct defined in msg/DigitalOutputCommand in the package baxter_core_msgs.
/**
  * the name of the output
 */
typedef struct baxter_core_msgs__msg__DigitalOutputCommand
{
  rosidl_runtime_c__String name;
  /// the value to set output
  bool value;
} baxter_core_msgs__msg__DigitalOutputCommand;

// Struct for a sequence of baxter_core_msgs__msg__DigitalOutputCommand.
typedef struct baxter_core_msgs__msg__DigitalOutputCommand__Sequence
{
  baxter_core_msgs__msg__DigitalOutputCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__DigitalOutputCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_OUTPUT_COMMAND__STRUCT_H_
