// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/NavigatorStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__STRUCT_H_

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
#include "baxter_core_msgs/msg/detail/navigator_state__struct.h"

/// Struct defined in msg/NavigatorStates in the package baxter_core_msgs.
/**
  * used when publishing multiple navigators
 */
typedef struct baxter_core_msgs__msg__NavigatorStates
{
  rosidl_runtime_c__String__Sequence names;
  baxter_core_msgs__msg__NavigatorState__Sequence states;
} baxter_core_msgs__msg__NavigatorStates;

// Struct for a sequence of baxter_core_msgs__msg__NavigatorStates.
typedef struct baxter_core_msgs__msg__NavigatorStates__Sequence
{
  baxter_core_msgs__msg__NavigatorStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__NavigatorStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATES__STRUCT_H_
