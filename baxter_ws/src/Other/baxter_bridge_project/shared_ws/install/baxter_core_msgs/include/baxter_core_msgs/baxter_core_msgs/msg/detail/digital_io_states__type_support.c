// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_core_msgs:msg/DigitalIOStates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_core_msgs/msg/detail/digital_io_states__rosidl_typesupport_introspection_c.h"
#include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_core_msgs/msg/detail/digital_io_states__functions.h"
#include "baxter_core_msgs/msg/detail/digital_io_states__struct.h"


// Include directives for member types
// Member `names`
#include "rosidl_runtime_c/string_functions.h"
// Member `states`
#include "baxter_core_msgs/msg/digital_io_state.h"
// Member `states`
#include "baxter_core_msgs/msg/detail/digital_io_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_core_msgs__msg__DigitalIOStates__init(message_memory);
}

void baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_fini_function(void * message_memory)
{
  baxter_core_msgs__msg__DigitalIOStates__fini(message_memory);
}

size_t baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__size_function__DigitalIOStates__names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_const_function__DigitalIOStates__names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_function__DigitalIOStates__names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__fetch_function__DigitalIOStates__names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_const_function__DigitalIOStates__names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__assign_function__DigitalIOStates__names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_function__DigitalIOStates__names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__resize_function__DigitalIOStates__names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__size_function__DigitalIOStates__states(
  const void * untyped_member)
{
  const baxter_core_msgs__msg__DigitalIOState__Sequence * member =
    (const baxter_core_msgs__msg__DigitalIOState__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_const_function__DigitalIOStates__states(
  const void * untyped_member, size_t index)
{
  const baxter_core_msgs__msg__DigitalIOState__Sequence * member =
    (const baxter_core_msgs__msg__DigitalIOState__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_function__DigitalIOStates__states(
  void * untyped_member, size_t index)
{
  baxter_core_msgs__msg__DigitalIOState__Sequence * member =
    (baxter_core_msgs__msg__DigitalIOState__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__fetch_function__DigitalIOStates__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const baxter_core_msgs__msg__DigitalIOState * item =
    ((const baxter_core_msgs__msg__DigitalIOState *)
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_const_function__DigitalIOStates__states(untyped_member, index));
  baxter_core_msgs__msg__DigitalIOState * value =
    (baxter_core_msgs__msg__DigitalIOState *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__assign_function__DigitalIOStates__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  baxter_core_msgs__msg__DigitalIOState * item =
    ((baxter_core_msgs__msg__DigitalIOState *)
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_function__DigitalIOStates__states(untyped_member, index));
  const baxter_core_msgs__msg__DigitalIOState * value =
    (const baxter_core_msgs__msg__DigitalIOState *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__resize_function__DigitalIOStates__states(
  void * untyped_member, size_t size)
{
  baxter_core_msgs__msg__DigitalIOState__Sequence * member =
    (baxter_core_msgs__msg__DigitalIOState__Sequence *)(untyped_member);
  baxter_core_msgs__msg__DigitalIOState__Sequence__fini(member);
  return baxter_core_msgs__msg__DigitalIOState__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_member_array[2] = {
  {
    "names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__DigitalIOStates, names),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__size_function__DigitalIOStates__names,  // size() function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_const_function__DigitalIOStates__names,  // get_const(index) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_function__DigitalIOStates__names,  // get(index) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__fetch_function__DigitalIOStates__names,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__assign_function__DigitalIOStates__names,  // assign(index, value) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__resize_function__DigitalIOStates__names  // resize(index) function pointer
  },
  {
    "states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__DigitalIOStates, states),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__size_function__DigitalIOStates__states,  // size() function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_const_function__DigitalIOStates__states,  // get_const(index) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__get_function__DigitalIOStates__states,  // get(index) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__fetch_function__DigitalIOStates__states,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__assign_function__DigitalIOStates__states,  // assign(index, value) function pointer
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__resize_function__DigitalIOStates__states  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_members = {
  "baxter_core_msgs__msg",  // message namespace
  "DigitalIOStates",  // message name
  2,  // number of fields
  sizeof(baxter_core_msgs__msg__DigitalIOStates),
  baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_member_array,  // message members
  baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_type_support_handle = {
  0,
  &baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, msg, DigitalIOStates)() {
  baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, msg, DigitalIOState)();
  if (!baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_core_msgs__msg__DigitalIOStates__rosidl_typesupport_introspection_c__DigitalIOStates_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
