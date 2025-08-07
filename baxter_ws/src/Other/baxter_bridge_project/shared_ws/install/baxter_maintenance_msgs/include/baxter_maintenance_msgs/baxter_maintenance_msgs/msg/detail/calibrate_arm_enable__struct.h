// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmEnable.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__STRUCT_H_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'uid'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_data__struct.h"

/// Struct defined in msg/CalibrateArmEnable in the package baxter_maintenance_msgs.
typedef struct baxter_maintenance_msgs__msg__CalibrateArmEnable
{
  bool is_enabled;
  rosidl_runtime_c__String uid;
  baxter_maintenance_msgs__msg__CalibrateArmData data;
} baxter_maintenance_msgs__msg__CalibrateArmEnable;

// Struct for a sequence of baxter_maintenance_msgs__msg__CalibrateArmEnable.
typedef struct baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence
{
  baxter_maintenance_msgs__msg__CalibrateArmEnable * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_maintenance_msgs__msg__CalibrateArmEnable__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__STRUCT_H_
