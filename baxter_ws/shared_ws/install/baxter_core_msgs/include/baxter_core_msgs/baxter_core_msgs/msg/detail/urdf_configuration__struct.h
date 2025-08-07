// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/URDFConfiguration.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'link'
// Member 'joint'
// Member 'urdf'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/URDFConfiguration in the package baxter_core_msgs.
/**
  * URDF Configuration
 */
typedef struct baxter_core_msgs__msg__URDFConfiguration
{
  /// time the message was created, serves as a sequence number
  builtin_interfaces__msg__Time time;
  /// time should be changed only when the content changes.
  /// parent link name
  rosidl_runtime_c__String link;
  /// joint to configure
  rosidl_runtime_c__String joint;
  /// link + joint + time uniquely identifies a configuration.
  /// XML or JSON-encoded URDF data. This should be a URDF fragment
  rosidl_runtime_c__String urdf;
} baxter_core_msgs__msg__URDFConfiguration;

// Struct for a sequence of baxter_core_msgs__msg__URDFConfiguration.
typedef struct baxter_core_msgs__msg__URDFConfiguration__Sequence
{
  baxter_core_msgs__msg__URDFConfiguration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__URDFConfiguration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__STRUCT_H_
