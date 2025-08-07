// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/BridgePublisher.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topic'
// Member 'user'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BridgePublisher in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__BridgePublisher
{
  rosidl_runtime_c__String topic;
  rosidl_runtime_c__String user;
  double time;
} baxter_core_msgs__msg__BridgePublisher;

// Struct for a sequence of baxter_core_msgs__msg__BridgePublisher.
typedef struct baxter_core_msgs__msg__BridgePublisher__Sequence
{
  baxter_core_msgs__msg__BridgePublisher * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__BridgePublisher__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__STRUCT_H_
