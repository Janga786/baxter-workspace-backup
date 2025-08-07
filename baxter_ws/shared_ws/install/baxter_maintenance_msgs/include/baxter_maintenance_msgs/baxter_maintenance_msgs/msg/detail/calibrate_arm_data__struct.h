// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmData.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__STRUCT_H_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CalibrateArmData in the package baxter_maintenance_msgs.
typedef struct baxter_maintenance_msgs__msg__CalibrateArmData
{
  bool suppress_write_to_file;
} baxter_maintenance_msgs__msg__CalibrateArmData;

// Struct for a sequence of baxter_maintenance_msgs__msg__CalibrateArmData.
typedef struct baxter_maintenance_msgs__msg__CalibrateArmData__Sequence
{
  baxter_maintenance_msgs__msg__CalibrateArmData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_maintenance_msgs__msg__CalibrateArmData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__STRUCT_H_
