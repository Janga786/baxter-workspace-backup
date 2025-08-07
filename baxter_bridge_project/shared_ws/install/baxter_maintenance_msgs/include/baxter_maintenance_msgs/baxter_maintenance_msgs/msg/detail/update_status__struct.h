// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_maintenance_msgs:msg/UpdateStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__STRUCT_H_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STS_IDLE'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_IDLE = 0
};

/// Constant 'STS_INVALID'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_INVALID = 1
};

/// Constant 'STS_BUSY'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_BUSY = 2
};

/// Constant 'STS_CANCELLED'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_CANCELLED = 3
};

/// Constant 'STS_ERR'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_ERR = 4
};

/// Constant 'STS_MOUNT_UPDATE'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_MOUNT_UPDATE = 5
};

/// Constant 'STS_VERIFY_UPDATE'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_VERIFY_UPDATE = 6
};

/// Constant 'STS_PREP_STAGING'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_PREP_STAGING = 7
};

/// Constant 'STS_MOUNT_STAGING'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_MOUNT_STAGING = 8
};

/// Constant 'STS_EXTRACT_UPDATE'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_EXTRACT_UPDATE = 9
};

/// Constant 'STS_LOAD_KEXEC'.
enum
{
  baxter_maintenance_msgs__msg__UpdateStatus__STS_LOAD_KEXEC = 10
};

// Include directives for member types
// Member 'long_description'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/UpdateStatus in the package baxter_maintenance_msgs.
/**
  * See the class UpdateRunner()
  * status:           One-word description of the current action being performed
  * long_description: Details pertaining to status if any.  Used for verbose error messages.
 */
typedef struct baxter_maintenance_msgs__msg__UpdateStatus
{
  uint16_t status;
  float progress;
  rosidl_runtime_c__String long_description;
} baxter_maintenance_msgs__msg__UpdateStatus;

// Struct for a sequence of baxter_maintenance_msgs__msg__UpdateStatus.
typedef struct baxter_maintenance_msgs__msg__UpdateStatus__Sequence
{
  baxter_maintenance_msgs__msg__UpdateStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_maintenance_msgs__msg__UpdateStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__STRUCT_H_
