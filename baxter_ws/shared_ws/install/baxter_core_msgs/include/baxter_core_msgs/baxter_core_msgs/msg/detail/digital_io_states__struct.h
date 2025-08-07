// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/DigitalIOStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATES__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATES__STRUCT_H_

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
#include "baxter_core_msgs/msg/detail/digital_io_state__struct.h"

/// Struct defined in msg/DigitalIOStates in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__DigitalIOStates
{
  rosidl_runtime_c__String__Sequence names;
  baxter_core_msgs__msg__DigitalIOState__Sequence states;
} baxter_core_msgs__msg__DigitalIOStates;

// Struct for a sequence of baxter_core_msgs__msg__DigitalIOStates.
typedef struct baxter_core_msgs__msg__DigitalIOStates__Sequence
{
  baxter_core_msgs__msg__DigitalIOStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__DigitalIOStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATES__STRUCT_H_
