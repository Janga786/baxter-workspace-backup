// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/AssemblyState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ESTOP_BUTTON_UNPRESSED'.
/**
  * Robot is not stopped and button is not pressed
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_BUTTON_UNPRESSED = 0
};

/// Constant 'ESTOP_BUTTON_PRESSED'.
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_BUTTON_PRESSED = 1
};

/// Constant 'ESTOP_BUTTON_UNKNOWN'.
/**
  * STATE_UNKNOWN when estop was asserted by a non-user source
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_BUTTON_UNKNOWN = 2
};

/// Constant 'ESTOP_BUTTON_RELEASED'.
/**
  * Was pressed, is now known to be released, but robot is still stopped.
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_BUTTON_RELEASED = 3
};

/// Constant 'ESTOP_SOURCE_NONE'.
/**
  * e-stop is not asserted
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_SOURCE_NONE = 0
};

/// Constant 'ESTOP_SOURCE_USER'.
/**
  * e-stop source is user input (the red button)
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_SOURCE_USER = 1
};

/// Constant 'ESTOP_SOURCE_UNKNOWN'.
/**
  * e-stop source is unknown
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_SOURCE_UNKNOWN = 2
};

/// Constant 'ESTOP_SOURCE_FAULT'.
/**
  * MotorController asserted e-stop in response to a joint fault
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_SOURCE_FAULT = 3
};

/// Constant 'ESTOP_SOURCE_BRAIN'.
/**
  * MotorController asserted e-stop in response to a lapse of the brain heartbeat
 */
enum
{
  baxter_core_msgs__msg__AssemblyState__ESTOP_SOURCE_BRAIN = 4
};

/// Struct defined in msg/AssemblyState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__AssemblyState
{
  /// true if enabled and ready to operate, e.g., not homing
  bool ready;
  /// true if enabled
  bool enabled;
  /// true if stopped -- e-stop asserted
  bool stopped;
  /// true if a component of the assembly has an error
  bool error;
  /// The following are specific to the robot top-level assembly:
  /// One of the following:
  uint8_t estop_button;
  /// If stopped is true, the source of the e-stop. One of the following:
  uint8_t estop_source;
} baxter_core_msgs__msg__AssemblyState;

// Struct for a sequence of baxter_core_msgs__msg__AssemblyState.
typedef struct baxter_core_msgs__msg__AssemblyState__Sequence
{
  baxter_core_msgs__msg__AssemblyState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__AssemblyState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__STRUCT_H_
