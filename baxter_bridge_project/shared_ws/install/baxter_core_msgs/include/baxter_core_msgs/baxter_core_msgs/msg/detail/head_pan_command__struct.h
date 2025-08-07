// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/HeadPanCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAX_SPEED_RATIO'.
static const float baxter_core_msgs__msg__HeadPanCommand__MAX_SPEED_RATIO = 1.0f;

/// Constant 'MIN_SPEED_RATIO'.
static const float baxter_core_msgs__msg__HeadPanCommand__MIN_SPEED_RATIO = 0.0f;

/// Constant 'REQUEST_PAN_DISABLE'.
/**
  * enable_pan_request is tristate: 0 = disable pan; 1 = enable pan; 2 = don't change pan
 */
enum
{
  baxter_core_msgs__msg__HeadPanCommand__REQUEST_PAN_DISABLE = 0
};

/// Constant 'REQUEST_PAN_ENABLE'.
enum
{
  baxter_core_msgs__msg__HeadPanCommand__REQUEST_PAN_ENABLE = 1
};

/// Constant 'REQUEST_PAN_VOID'.
enum
{
  baxter_core_msgs__msg__HeadPanCommand__REQUEST_PAN_VOID = 2
};

/// Struct defined in msg/HeadPanCommand in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__HeadPanCommand
{
  /// radians for target, 0 str
  float target;
  /// Percentage of max speed
  float speed_ratio;
  /// override automatic pan enable/disable
  uint8_t enable_pan_request;
} baxter_core_msgs__msg__HeadPanCommand;

// Struct for a sequence of baxter_core_msgs__msg__HeadPanCommand.
typedef struct baxter_core_msgs__msg__HeadPanCommand__Sequence
{
  baxter_core_msgs__msg__HeadPanCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__HeadPanCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__STRUCT_H_
