// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/EndpointState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATE__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__struct.h"

/// Struct defined in msg/EndpointState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__EndpointState
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist twist;
  geometry_msgs__msg__Wrench wrench;
} baxter_core_msgs__msg__EndpointState;

// Struct for a sequence of baxter_core_msgs__msg__EndpointState.
typedef struct baxter_core_msgs__msg__EndpointState__Sequence
{
  baxter_core_msgs__msg__EndpointState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__EndpointState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ENDPOINT_STATE__STRUCT_H_
