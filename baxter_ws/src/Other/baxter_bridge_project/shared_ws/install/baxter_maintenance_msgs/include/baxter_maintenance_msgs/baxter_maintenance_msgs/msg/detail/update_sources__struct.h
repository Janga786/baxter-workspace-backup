// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_maintenance_msgs:msg/UpdateSources.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__STRUCT_H_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'uuid'
#include "rosidl_runtime_c/string.h"
// Member 'sources'
#include "baxter_maintenance_msgs/msg/detail/update_source__struct.h"

/// Struct defined in msg/UpdateSources in the package baxter_maintenance_msgs.
typedef struct baxter_maintenance_msgs__msg__UpdateSources
{
  rosidl_runtime_c__String uuid;
  baxter_maintenance_msgs__msg__UpdateSource__Sequence sources;
} baxter_maintenance_msgs__msg__UpdateSources;

// Struct for a sequence of baxter_maintenance_msgs__msg__UpdateSources.
typedef struct baxter_maintenance_msgs__msg__UpdateSources__Sequence
{
  baxter_maintenance_msgs__msg__UpdateSources * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_maintenance_msgs__msg__UpdateSources__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__STRUCT_H_
