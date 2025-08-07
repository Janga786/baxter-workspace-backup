// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_core_msgs/msg/detail/camera_settings__rosidl_typesupport_introspection_c.h"
#include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_core_msgs/msg/detail/camera_settings__functions.h"
#include "baxter_core_msgs/msg/detail/camera_settings__struct.h"


// Include directives for member types
// Member `controls`
#include "baxter_core_msgs/msg/camera_control.h"
// Member `controls`
#include "baxter_core_msgs/msg/detail/camera_control__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_core_msgs__msg__CameraSettings__init(message_memory);
}

void baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_fini_function(void * message_memory)
{
  baxter_core_msgs__msg__CameraSettings__fini(message_memory);
}

size_t baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__size_function__CameraSettings__controls(
  const void * untyped_member)
{
  const baxter_core_msgs__msg__CameraControl__Sequence * member =
    (const baxter_core_msgs__msg__CameraControl__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__get_const_function__CameraSettings__controls(
  const void * untyped_member, size_t index)
{
  const baxter_core_msgs__msg__CameraControl__Sequence * member =
    (const baxter_core_msgs__msg__CameraControl__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__get_function__CameraSettings__controls(
  void * untyped_member, size_t index)
{
  baxter_core_msgs__msg__CameraControl__Sequence * member =
    (baxter_core_msgs__msg__CameraControl__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__fetch_function__CameraSettings__controls(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const baxter_core_msgs__msg__CameraControl * item =
    ((const baxter_core_msgs__msg__CameraControl *)
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__get_const_function__CameraSettings__controls(untyped_member, index));
  baxter_core_msgs__msg__CameraControl * value =
    (baxter_core_msgs__msg__CameraControl *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__assign_function__CameraSettings__controls(
  void * untyped_member, size_t index, const void * untyped_value)
{
  baxter_core_msgs__msg__CameraControl * item =
    ((baxter_core_msgs__msg__CameraControl *)
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__get_function__CameraSettings__controls(untyped_member, index));
  const baxter_core_msgs__msg__CameraControl * value =
    (const baxter_core_msgs__msg__CameraControl *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__resize_function__CameraSettings__controls(
  void * untyped_member, size_t size)
{
  baxter_core_msgs__msg__CameraControl__Sequence * member =
    (baxter_core_msgs__msg__CameraControl__Sequence *)(untyped_member);
  baxter_core_msgs__msg__CameraControl__Sequence__fini(member);
  return baxter_core_msgs__msg__CameraControl__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_member_array[4] = {
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__CameraSettings, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__CameraSettings, height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__CameraSettings, fps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "controls",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__CameraSettings, controls),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__size_function__CameraSettings__controls,  // size() function pointer
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__get_const_function__CameraSettings__controls,  // get_const(index) function pointer
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__get_function__CameraSettings__controls,  // get(index) function pointer
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__fetch_function__CameraSettings__controls,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__assign_function__CameraSettings__controls,  // assign(index, value) function pointer
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__resize_function__CameraSettings__controls  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_members = {
  "baxter_core_msgs__msg",  // message namespace
  "CameraSettings",  // message name
  4,  // number of fields
  sizeof(baxter_core_msgs__msg__CameraSettings),
  baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_member_array,  // message members
  baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_type_support_handle = {
  0,
  &baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, msg, CameraSettings)() {
  baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, msg, CameraControl)();
  if (!baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_core_msgs__msg__CameraSettings__rosidl_typesupport_introspection_c__CameraSettings_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
