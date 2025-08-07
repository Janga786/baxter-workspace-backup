// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/EndpointStates.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATES__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATES__STRUCT_H_

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
#include "baxter_core_msgs/msg/detail/endpoint_state__struct.h"

/// Struct defined in msg/EndpointStates in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__EndpointStates
{
  rosidl_runtime_c__String__Sequence names;
  baxter_core_msgs__msg__EndpointState__Sequence states;
} baxter_core_msgs__msg__EndpointStates;

// Struct for a sequence of baxter_core_msgs__msg__EndpointStates.
typedef struct baxter_core_msgs__msg__EndpointStates__Sequence
{
  baxter_core_msgs__msg__EndpointStates * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__EndpointStates__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATES__STRUCT_H_
