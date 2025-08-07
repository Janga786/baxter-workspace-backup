// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:srv/BridgePublishersForce.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__STRUCT_H_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'left_user'
// Member 'right_user'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/BridgePublishersForce in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__BridgePublishersForce_Request
{
  rosidl_runtime_c__String left_user;
  rosidl_runtime_c__String right_user;
} baxter_core_msgs__srv__BridgePublishersForce_Request;

// Struct for a sequence of baxter_core_msgs__srv__BridgePublishersForce_Request.
typedef struct baxter_core_msgs__srv__BridgePublishersForce_Request__Sequence
{
  baxter_core_msgs__srv__BridgePublishersForce_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__BridgePublishersForce_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/BridgePublishersForce in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__BridgePublishersForce_Response
{
  uint8_t structure_needs_at_least_one_member;
} baxter_core_msgs__srv__BridgePublishersForce_Response;

// Struct for a sequence of baxter_core_msgs__srv__BridgePublishersForce_Response.
typedef struct baxter_core_msgs__srv__BridgePublishersForce_Response__Sequence
{
  baxter_core_msgs__srv__BridgePublishersForce_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__BridgePublishersForce_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__STRUCT_H_
