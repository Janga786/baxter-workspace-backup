// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_maintenance_msgs:msg/TareData.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__STRUCT_H_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TareData in the package baxter_maintenance_msgs.
typedef struct baxter_maintenance_msgs__msg__TareData
{
  bool tune_gravity_spring;
} baxter_maintenance_msgs__msg__TareData;

// Struct for a sequence of baxter_maintenance_msgs__msg__TareData.
typedef struct baxter_maintenance_msgs__msg__TareData__Sequence
{
  baxter_maintenance_msgs__msg__TareData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_maintenance_msgs__msg__TareData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__TARE_DATA__STRUCT_H_
