// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'controls'
#include "baxter_core_msgs/msg/detail/camera_control__struct.h"

/// Struct defined in msg/CameraSettings in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__CameraSettings
{
  int32_t width;
  int32_t height;
  float fps;
  baxter_core_msgs__msg__CameraControl__Sequence controls;
} baxter_core_msgs__msg__CameraSettings;

// Struct for a sequence of baxter_core_msgs__msg__CameraSettings.
typedef struct baxter_core_msgs__msg__CameraSettings__Sequence
{
  baxter_core_msgs__msg__CameraSettings * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__CameraSettings__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__STRUCT_H_
