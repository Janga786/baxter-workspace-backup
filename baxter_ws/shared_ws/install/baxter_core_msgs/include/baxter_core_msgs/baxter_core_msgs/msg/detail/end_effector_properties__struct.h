// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:msg/EndEffectorProperties.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__STRUCT_H_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NO_GRIPPER'.
/**
  * End Effector type enumeration, for the UI:
 */
enum
{
  baxter_core_msgs__msg__EndEffectorProperties__NO_GRIPPER = 0
};

/// Constant 'SUCTION_CUP_GRIPPER'.
enum
{
  baxter_core_msgs__msg__EndEffectorProperties__SUCTION_CUP_GRIPPER = 1
};

/// Constant 'ELECTRIC_GRIPPER'.
enum
{
  baxter_core_msgs__msg__EndEffectorProperties__ELECTRIC_GRIPPER = 2
};

/// Constant 'PASSIVE_GRIPPER'.
enum
{
  baxter_core_msgs__msg__EndEffectorProperties__PASSIVE_GRIPPER = 3
};

// Include directives for member types
// Member 'manufacturer'
// Member 'product'
// Member 'serial_number'
// Member 'hardware_rev'
// Member 'firmware_rev'
// Member 'firmware_date'
// Member 'properties'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/EndEffectorProperties in the package baxter_core_msgs.
typedef struct baxter_core_msgs__msg__EndEffectorProperties
{
  /// EndEffectorId
  uint32_t id;
  uint8_t ui_type;
  /// Manufacturer name
  rosidl_runtime_c__String manufacturer;
  /// Product name
  rosidl_runtime_c__String product;
  /// Serial number, optional
  rosidl_runtime_c__String serial_number;
  /// Hardware revision, optional
  rosidl_runtime_c__String hardware_rev;
  /// Firmware revision, optional
  rosidl_runtime_c__String firmware_rev;
  /// Firmware date, optional
  rosidl_runtime_c__String firmware_date;
  /// End Effector Capabilities
  /// true if the gripper has calibration
  bool has_calibration;
  /// true if the gripper has grip/release control
  bool controls_grip;
  /// true if the gripper has grip sense
  bool senses_grip;
  /// true if the gripper has reverse-grip mode
  bool reverses_grip;
  /// true if the gripper has force control
  bool controls_force;
  /// true if the gripper has force sense
  bool senses_force;
  /// true if the gripper has position control
  bool controls_position;
  /// true if the gripper has position sense
  bool senses_position;
  /// JSON; other properties
  rosidl_runtime_c__String properties;
} baxter_core_msgs__msg__EndEffectorProperties;

// Struct for a sequence of baxter_core_msgs__msg__EndEffectorProperties.
typedef struct baxter_core_msgs__msg__EndEffectorProperties__Sequence
{
  baxter_core_msgs__msg__EndEffectorProperties * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__msg__EndEffectorProperties__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__STRUCT_H_
