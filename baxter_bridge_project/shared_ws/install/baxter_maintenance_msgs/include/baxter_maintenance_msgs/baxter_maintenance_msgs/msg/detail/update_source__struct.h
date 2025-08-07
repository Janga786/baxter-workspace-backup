// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_maintenance_msgs:msg/UpdateSource.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__STRUCT_H_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'devname'
// Member 'filename'
// Member 'version'
// Member 'uuid'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/UpdateSource in the package baxter_maintenance_msgs.
typedef struct baxter_maintenance_msgs__msg__UpdateSource
{
  rosidl_runtime_c__String devname;
  rosidl_runtime_c__String filename;
  rosidl_runtime_c__String version;
  rosidl_runtime_c__String uuid;
} baxter_maintenance_msgs__msg__UpdateSource;

// Struct for a sequence of baxter_maintenance_msgs__msg__UpdateSource.
typedef struct baxter_maintenance_msgs__msg__UpdateSource__Sequence
{
  baxter_maintenance_msgs__msg__UpdateSource * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_maintenance_msgs__msg__UpdateSource__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__STRUCT_H_
