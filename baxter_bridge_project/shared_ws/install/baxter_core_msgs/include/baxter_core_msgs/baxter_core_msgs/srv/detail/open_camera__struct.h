// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from baxter_core_msgs:srv/OpenCamera.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__OPEN_CAMERA__STRUCT_H_
#define BAXTER_CORE_MSGS__SRV__DETAIL__OPEN_CAMERA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'settings'
#include "baxter_core_msgs/msg/detail/camera_settings__struct.h"

/// Struct defined in srv/OpenCamera in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__OpenCamera_Request
{
  rosidl_runtime_c__String name;
  baxter_core_msgs__msg__CameraSettings settings;
} baxter_core_msgs__srv__OpenCamera_Request;

// Struct for a sequence of baxter_core_msgs__srv__OpenCamera_Request.
typedef struct baxter_core_msgs__srv__OpenCamera_Request__Sequence
{
  baxter_core_msgs__srv__OpenCamera_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__OpenCamera_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/OpenCamera in the package baxter_core_msgs.
typedef struct baxter_core_msgs__srv__OpenCamera_Response
{
  int32_t err;
} baxter_core_msgs__srv__OpenCamera_Response;

// Struct for a sequence of baxter_core_msgs__srv__OpenCamera_Response.
typedef struct baxter_core_msgs__srv__OpenCamera_Response__Sequence
{
  baxter_core_msgs__srv__OpenCamera_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} baxter_core_msgs__srv__OpenCamera_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__OPEN_CAMERA__STRUCT_H_
