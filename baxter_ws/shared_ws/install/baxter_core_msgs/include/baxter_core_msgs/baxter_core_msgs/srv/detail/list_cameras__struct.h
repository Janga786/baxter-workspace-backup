// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:srv/ListCameras.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__STRUCT_H_
#define BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ListCameras in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__ListCameras_Request
{
  uint8_t structure_needs_at_least_one_member;
} baxter_core_msgs__srv__ListCameras_Request;

// Struct for a sequence of baxter_core_msgs__srv__ListCameras_Request.
typedef struct baxter_core_msgs__srv__ListCameras_Request__Sequence
{
  baxter_core_msgs__srv__ListCameras_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__ListCameras_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cameras'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ListCameras in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__ListCameras_Response
{
  rosidl_runtime_c__String__Sequence cameras;
} baxter_core_msgs__srv__ListCameras_Response;

// Struct for a sequence of baxter_core_msgs__srv__ListCameras_Response.
typedef struct baxter_core_msgs__srv__ListCameras_Response__Sequence
{
  baxter_core_msgs__srv__ListCameras_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__ListCameras_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__STRUCT_H_
