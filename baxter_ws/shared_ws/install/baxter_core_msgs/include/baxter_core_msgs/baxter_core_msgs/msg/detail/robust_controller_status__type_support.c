// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_core_msgs:msg/RobustControllerStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_core_msgs/msg/detail/robust_controller_status__rosidl_typesupport_introspection_c.h"
#include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_core_msgs/msg/detail/robust_controller_status__functions.h"
#include "baxter_core_msgs/msg/detail/robust_controller_status__struct.h"


// Include directives for member types
// Member `control_uid`
// Member `error_codes`
// Member `labels`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_core_msgs__msg__RobustControllerStatus__init(message_memory);
}

void baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_fini_function(void * message_memory)
{
  baxter_core_msgs__msg__RobustControllerStatus__fini(message_memory);
}

size_t baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__size_function__RobustControllerStatus__error_codes(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_const_function__RobustControllerStatus__error_codes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_function__RobustControllerStatus__error_codes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__fetch_function__RobustControllerStatus__error_codes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_const_function__RobustControllerStatus__error_codes(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__assign_function__RobustControllerStatus__error_codes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_function__RobustControllerStatus__error_codes(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__resize_function__RobustControllerStatus__error_codes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__size_function__RobustControllerStatus__labels(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_const_function__RobustControllerStatus__labels(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_function__RobustControllerStatus__labels(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__fetch_function__RobustControllerStatus__labels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_const_function__RobustControllerStatus__labels(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__assign_function__RobustControllerStatus__labels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_function__RobustControllerStatus__labels(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__resize_function__RobustControllerStatus__labels(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_member_array[6] = {
  {
    "is_enabled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__RobustControllerStatus, is_enabled),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "complete",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__RobustControllerStatus, complete),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_uid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__RobustControllerStatus, control_uid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timed_out",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__RobustControllerStatus, timed_out),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_codes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__RobustControllerStatus, error_codes),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__size_function__RobustControllerStatus__error_codes,  // size() function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_const_function__RobustControllerStatus__error_codes,  // get_const(index) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_function__RobustControllerStatus__error_codes,  // get(index) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__fetch_function__RobustControllerStatus__error_codes,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__assign_function__RobustControllerStatus__error_codes,  // assign(index, value) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__resize_function__RobustControllerStatus__error_codes  // resize(index) function pointer
  },
  {
    "labels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__RobustControllerStatus, labels),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__size_function__RobustControllerStatus__labels,  // size() function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_const_function__RobustControllerStatus__labels,  // get_const(index) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__get_function__RobustControllerStatus__labels,  // get(index) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__fetch_function__RobustControllerStatus__labels,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__assign_function__RobustControllerStatus__labels,  // assign(index, value) function pointer
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__resize_function__RobustControllerStatus__labels  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_members = {
  "baxter_core_msgs__msg",  // message namespace
  "RobustControllerStatus",  // message name
  6,  // number of fields
  sizeof(baxter_core_msgs__msg__RobustControllerStatus),
  baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_member_array,  // message members
  baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_type_support_handle = {
  0,
  &baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, msg, RobustControllerStatus)() {
  if (!baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_core_msgs__msg__RobustControllerStatus__rosidl_typesupport_introspection_c__RobustControllerStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
