// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:srv/BridgePublishersAuth.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__STRUCT_H_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topic'
// Member 'user'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/BridgePublishersAuth in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__BridgePublishersAuth_Request
{
  rosidl_runtime_c__String topic;
  rosidl_runtime_c__String user;
} baxter_core_msgs__srv__BridgePublishersAuth_Request;

// Struct for a sequence of baxter_core_msgs__srv__BridgePublishersAuth_Request.
typedef struct baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence
{
  baxter_core_msgs__srv__BridgePublishersAuth_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__BridgePublishersAuth_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'publishers'
#include "baxter_core_msgs/msg/detail/bridge_publisher__struct.h"
// Member 'forced_left'
// Member 'forced_right'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/BridgePublishersAuth in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__BridgePublishersAuth_Response
{
  baxter_core_msgs__msg__BridgePublisher__Sequence publishers;
  rosidl_runtime_c__String forced_left;
  rosidl_runtime_c__String forced_right;
} baxter_core_msgs__srv__BridgePublishersAuth_Response;

// Struct for a sequence of baxter_core_msgs__srv__BridgePublishersAuth_Response.
typedef struct baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence
{
  baxter_core_msgs__srv__BridgePublishersAuth_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__BridgePublishersAuth_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__STRUCT_H_
