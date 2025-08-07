// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/RobustControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NOT_COMPLETE'.
enum
{
  baxter_core_msgs__msg__RobustControllerStatus__NOT_COMPLETE = 0l
};

/// Constant 'COMPLETE_W_FAILURE'.
enum
{
  baxter_core_msgs__msg__RobustControllerStatus__COMPLETE_W_FAILURE = 1l
};

/// Constant 'COMPLETE_W_SUCCESS'.
enum
{
  baxter_core_msgs__msg__RobustControllerStatus__COMPLETE_W_SUCCESS = 2l
};

// Include directives for member types
// Member 'control_uid'
// Member 'error_codes'
// Member 'labels'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RobustControllerStatus in the package baxter_core_msgs.
/**
  * True if the RC is enabled and running, false if not.
 */
typedef struct baxter_core_msgs__msg__RobustControllerStatus
{
  bool is_enabled;
  /// The state of the RC with respect to its completion goal.  One of
  /// NOT_COMPLETE, COMPLETE_W_FAILURE, or COMPLETE_W_SUCCESS
  int32_t complete;
  /// Identifies the sender of the Enable message that the RC is using for its
  /// commands.  This should correspond to the "uid" field of a recently published
  /// RC *Enable message.
  rosidl_runtime_c__String control_uid;
  /// Set to true when the RC self-disables as a result of too much time elapsing
  /// without receiving an Enable message.
  bool timed_out;
  /// A list of relevant error codes.  Error codes are defined by the individual
  /// robust controllers, consult a robust controller's documentation to see what
  /// error codes it generates.
  rosidl_runtime_c__String__Sequence error_codes;
  /// A list of current labels for the RC's current state. For example, "fastapproach",
  /// "slowapproach", etc. Used primarily for the blended RCs, other RCs can leave this
  /// blank. This will probably contains just one label, but it could contain multiple labels
  /// in the future.
  rosidl_runtime_c__String__Sequence labels;
} baxter_core_msgs__msg__RobustControllerStatus;

// Struct for a sequence of baxter_core_msgs__msg__RobustControllerStatus.
typedef struct baxter_core_msgs__msg__RobustControllerStatus__Sequence
{
  baxter_core_msgs__msg__RobustControllerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__RobustControllerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__STRUCT_H_
