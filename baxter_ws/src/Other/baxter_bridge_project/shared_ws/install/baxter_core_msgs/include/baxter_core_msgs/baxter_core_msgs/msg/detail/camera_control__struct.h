// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/CameraControl.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CAMERA_CONTROL_EXPOSURE'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_EXPOSURE = 100l
};

/// Constant 'CAMERA_CONTROL_GAIN'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_GAIN = 101l
};

/// Constant 'CAMERA_CONTROL_WHITE_BALANCE_R'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_WHITE_BALANCE_R = 102l
};

/// Constant 'CAMERA_CONTROL_WHITE_BALANCE_G'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_WHITE_BALANCE_G = 103l
};

/// Constant 'CAMERA_CONTROL_WHITE_BALANCE_B'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_WHITE_BALANCE_B = 104l
};

/// Constant 'CAMERA_CONTROL_WINDOW_X'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_WINDOW_X = 105l
};

/// Constant 'CAMERA_CONTROL_WINDOW_Y'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_WINDOW_Y = 106l
};

/// Constant 'CAMERA_CONTROL_FLIP'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_FLIP = 107l
};

/// Constant 'CAMERA_CONTROL_MIRROR'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_MIRROR = 108l
};

/// Constant 'CAMERA_CONTROL_RESOLUTION_HALF'.
enum
{
  baxter_core_msgs__msg__CameraControl__CAMERA_CONTROL_RESOLUTION_HALF = 109l
};

/// Struct defined in msg/CameraControl in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__CameraControl
{
  int32_t id;
  int32_t value;
} baxter_core_msgs__msg__CameraControl;

// Struct for a sequence of baxter_core_msgs__msg__CameraControl.
typedef struct baxter_core_msgs__msg__CameraControl__Sequence
{
  baxter_core_msgs__msg__CameraControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__CameraControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_CONTROL__STRUCT_H_
