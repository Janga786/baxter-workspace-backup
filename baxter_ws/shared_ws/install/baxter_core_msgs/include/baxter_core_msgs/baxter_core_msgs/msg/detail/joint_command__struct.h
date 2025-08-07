// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'POSITION_MODE'.
enum
{
  baxter_core_msgs__msg__JointCommand__POSITION_MODE = 1l
};

/// Constant 'VELOCITY_MODE'.
enum
{
  baxter_core_msgs__msg__JointCommand__VELOCITY_MODE = 2l
};

/// Constant 'TORQUE_MODE'.
enum
{
  baxter_core_msgs__msg__JointCommand__TORQUE_MODE = 3l
};

/// Constant 'RAW_POSITION_MODE'.
enum
{
  baxter_core_msgs__msg__JointCommand__RAW_POSITION_MODE = 4l
};

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'names'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/JointCommand in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__JointCommand
{
  int32_t mode;
  rosidl_runtime_c__double__Sequence command;
  rosidl_runtime_c__String__Sequence names;
} baxter_core_msgs__msg__JointCommand;

// Struct for a sequence of baxter_core_msgs__msg__JointCommand.
typedef struct baxter_core_msgs__msg__JointCommand__Sequence
{
  baxter_core_msgs__msg__JointCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__JointCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__JOINT_COMMAND__STRUCT_H_
