// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/EndEffectorState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_FALSE'.
/**
  * The following State fields are tristate: 0 = false; 1 = true; 2 = unknown/unsupported
 */
enum
{
  baxter_core_msgs__msg__EndEffectorState__STATE_FALSE = 0
};

/// Constant 'STATE_TRUE'.
enum
{
  baxter_core_msgs__msg__EndEffectorState__STATE_TRUE = 1
};

/// Constant 'STATE_UNKNOWN'.
enum
{
  baxter_core_msgs__msg__EndEffectorState__STATE_UNKNOWN = 2
};

/// Constant 'POSITION_CLOSED'.
static const float baxter_core_msgs__msg__EndEffectorState__POSITION_CLOSED = 0.0f;

/// Constant 'POSITION_OPEN'.
static const float baxter_core_msgs__msg__EndEffectorState__POSITION_OPEN = 100.0f;

/// Constant 'FORCE_MIN'.
static const float baxter_core_msgs__msg__EndEffectorState__FORCE_MIN = 0.0f;

/// Constant 'FORCE_MAX'.
static const float baxter_core_msgs__msg__EndEffectorState__FORCE_MAX = 100.0f;

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'state'
// Member 'command'
// Member 'command_sender'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/EndEffectorState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__EndEffectorState
{
  /// time when state was updated
  builtin_interfaces__msg__Time timestamp;
  /// EndEffectorId
  uint32_t id;
  /// true if enabled
  uint8_t enabled;
  /// true if calibration has completed
  uint8_t calibrated;
  /// true if ready for another command
  uint8_t ready;
  /// true if moving
  uint8_t moving;
  /// true if gripping
  uint8_t gripping;
  /// true if GRIP/GOTO/SET was commanded and the gripper reaches the end of travel
  uint8_t missed;
  /// true if the gripper is in an error state
  uint8_t error;
  /// true if the gripper is in reverse mode
  uint8_t reverse;
  /// position as a percentage of the max position; 0=closed - 100=open
  float position;
  /// force as a percentage of max force; 0=none - 100=max
  float force;
  /// JSON: other state information
  rosidl_runtime_c__String state;
  /// from the last command message
  rosidl_runtime_c__String command;
  rosidl_runtime_c__String command_sender;
  uint32_t command_sequence;
} baxter_core_msgs__msg__EndEffectorState;

// Struct for a sequence of baxter_core_msgs__msg__EndEffectorState.
typedef struct baxter_core_msgs__msg__EndEffectorState__Sequence
{
  baxter_core_msgs__msg__EndEffectorState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__EndEffectorState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__STRUCT_H_
