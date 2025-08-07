// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/EndEffectorCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CMD_NO_OP'.
/**
  * Well known commands:
 */
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_NO_OP = "no_op";

/// Constant 'CMD_SET'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_SET = "set";

/// Constant 'CMD_CONFIGURE'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_CONFIGURE = "configure";

/// Constant 'CMD_REBOOT'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_REBOOT = "reboot";

/// Constant 'CMD_RESET'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_RESET = "reset";

/// Constant 'CMD_CALIBRATE'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_CALIBRATE = "calibrate";

/// Constant 'CMD_CLEAR_CALIBRATION'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_CLEAR_CALIBRATION = "clear_calibration";

/// Constant 'CMD_PREPARE_TO_GRIP'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_PREPARE_TO_GRIP = "prepare_to_grip";

/// Constant 'CMD_GRIP'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_GRIP = "grip";

/// Constant 'CMD_RELEASE'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_RELEASE = "release";

/// Constant 'CMD_GO'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_GO = "go";

/// Constant 'CMD_STOP'.
static const char * const baxter_core_msgs__msg__EndEffectorCommand__CMD_STOP = "stop";

// Include directives for member types
// Member 'command'
// Member 'args'
// Member 'sender'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/EndEffectorCommand in the package baxter_core_msgs.
/**
  * Command to be sent to an end effector
 */
typedef struct baxter_core_msgs__msg__EndEffectorCommand
{
  /// target end effector id
  uint32_t id;
  /// operation to perform
  rosidl_runtime_c__String command;
  /// JSON arguments to the command
  rosidl_runtime_c__String args;
  /// optional identifier, returned in state when the command is handled
  rosidl_runtime_c__String sender;
  /// optional sequence number, return in state when the command is handled
  uint32_t sequence;
} baxter_core_msgs__msg__EndEffectorCommand;

// Struct for a sequence of baxter_core_msgs__msg__EndEffectorCommand.
typedef struct baxter_core_msgs__msg__EndEffectorCommand__Sequence
{
  baxter_core_msgs__msg__EndEffectorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__EndEffectorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__STRUCT_H_
