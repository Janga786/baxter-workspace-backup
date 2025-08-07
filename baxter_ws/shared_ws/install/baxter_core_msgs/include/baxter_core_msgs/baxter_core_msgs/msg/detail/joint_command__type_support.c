// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_core_msgs:msg/JointCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_core_msgs/msg/detail/joint_command__rosidl_typesupport_introspection_c.h"
#include "baxter_core_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_core_msgs/msg/detail/joint_command__functions.h"
#include "baxter_core_msgs/msg/detail/joint_command__struct.h"


// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `names`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_core_msgs__msg__JointCommand__init(message_memory);
}

void baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_fini_function(void * message_memory)
{
  baxter_core_msgs__msg__JointCommand__fini(message_memory);
}

size_t baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__command(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__command(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__command(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__command(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__command(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__command(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__command(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__command(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_member_array[3] = {
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__JointCommand, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__JointCommand, command),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__command,  // size() function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__command,  // get_const(index) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__command,  // get(index) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__command,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__command,  // assign(index, value) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__command  // resize(index) function pointer
  },
  {
    "names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs__msg__JointCommand, names),  // bytes offset in struct
    NULL,  // default value
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__size_function__JointCommand__names,  // size() function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_const_function__JointCommand__names,  // get_const(index) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__get_function__JointCommand__names,  // get(index) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__fetch_function__JointCommand__names,  // fetch(index, &value) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__assign_function__JointCommand__names,  // assign(index, value) function pointer
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__resize_function__JointCommand__names  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_members = {
  "baxter_core_msgs__msg",  // message namespace
  "JointCommand",  // message name
  3,  // number of fields
  sizeof(baxter_core_msgs__msg__JointCommand),
  baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_member_array,  // message members
  baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle = {
  0,
  &baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_core_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_core_msgs, msg, JointCommand)() {
  if (!baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle.typesupport_identifier) {
    baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_core_msgs__msg__JointCommand__rosidl_typesupport_introspection_c__JointCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
