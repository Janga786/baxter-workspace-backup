// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/DigitalIOState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'OFF'.
enum
{
  baxter_core_msgs__msg__DigitalIOState__OFF = 0
};

/// Constant 'ON'.
enum
{
  baxter_core_msgs__msg__DigitalIOState__ON = 1
};

/// Constant 'PRESSED'.
enum
{
  baxter_core_msgs__msg__DigitalIOState__PRESSED = 1
};

/// Constant 'UNPRESSED'.
enum
{
  baxter_core_msgs__msg__DigitalIOState__UNPRESSED = 0
};

/// Struct defined in msg/DigitalIOState in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__DigitalIOState
{
  int8_t state;
  bool is_input_only;
} baxter_core_msgs__msg__DigitalIOState;

// Struct for a sequence of baxter_core_msgs__msg__DigitalIOState.
typedef struct baxter_core_msgs__msg__DigitalIOState__Sequence
{
  baxter_core_msgs__msg__DigitalIOState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__DigitalIOState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__DIGITAL_IO_STATE__STRUCT_H_
