// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_maintenance_msgs:msg/UpdateSources.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_maintenance_msgs/msg/detail/update_sources__rosidl_typesupport_introspection_c.h"
#include "baxter_maintenance_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_maintenance_msgs/msg/detail/update_sources__functions.h"
#include "baxter_maintenance_msgs/msg/detail/update_sources__struct.h"


// Include directives for member types
// Member `uuid`
#include "rosidl_runtime_c/string_functions.h"
// Member `sources`
#include "baxter_maintenance_msgs/msg/update_source.h"
// Member `sources`
#include "baxter_maintenance_msgs/msg/detail/update_source__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_maintenance_msgs__msg__UpdateSources__init(message_memory);
}

void baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_fini_function(void * message_memory)
{
  baxter_maintenance_msgs__msg__UpdateSources__fini(message_memory);
}

size_t baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__size_function__UpdateSources__sources(
  const void * untyped_member)
{
  const baxter_maintenance_msgs__msg__UpdateSource__Sequence * member =
    (const baxter_maintenance_msgs__msg__UpdateSource__Sequence *)(untyped_member);
  return member->size;
}

const void * baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__get_const_function__UpdateSources__sources(
  const void * untyped_member, size_t index)
{
  const baxter_maintenance_msgs__msg__UpdateSource__Sequence * member =
    (const baxter_maintenance_msgs__msg__UpdateSource__Sequence *)(untyped_member);
  return &member->data[index];
}

void * baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__get_function__UpdateSources__sources(
  void * untyped_member, size_t index)
{
  baxter_maintenance_msgs__msg__UpdateSource__Sequence * member =
    (baxter_maintenance_msgs__msg__UpdateSource__Sequence *)(untyped_member);
  return &member->data[index];
}

void baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__fetch_function__UpdateSources__sources(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const baxter_maintenance_msgs__msg__UpdateSource * item =
    ((const baxter_maintenance_msgs__msg__UpdateSource *)
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__get_const_function__UpdateSources__sources(untyped_member, index));
  baxter_maintenance_msgs__msg__UpdateSource * value =
    (baxter_maintenance_msgs__msg__UpdateSource *)(untyped_value);
  *value = *item;
}

void baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__assign_function__UpdateSources__sources(
  void * untyped_member, size_t index, const void * untyped_value)
{
  baxter_maintenance_msgs__msg__UpdateSource * item =
    ((baxter_maintenance_msgs__msg__UpdateSource *)
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__get_function__UpdateSources__sources(untyped_member, index));
  const baxter_maintenance_msgs__msg__UpdateSource * value =
    (const baxter_maintenance_msgs__msg__UpdateSource *)(untyped_value);
  *item = *value;
}

bool baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__resize_function__UpdateSources__sources(
  void * untyped_member, size_t size)
{
  baxter_maintenance_msgs__msg__UpdateSource__Sequence * member =
    (baxter_maintenance_msgs__msg__UpdateSource__Sequence *)(untyped_member);
  baxter_maintenance_msgs__msg__UpdateSource__Sequence__fini(member);
  return baxter_maintenance_msgs__msg__UpdateSource__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_member_array[2] = {
  {
    "uuid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs__msg__UpdateSources, uuid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sources",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs__msg__UpdateSources, sources),  // bytes offset in struct
    NULL,  // default value
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__size_function__UpdateSources__sources,  // size() function pointer
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__get_const_function__UpdateSources__sources,  // get_const(index) function pointer
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__get_function__UpdateSources__sources,  // get(index) function pointer
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__fetch_function__UpdateSources__sources,  // fetch(index, &value) function pointer
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__assign_function__UpdateSources__sources,  // assign(index, value) function pointer
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__resize_function__UpdateSources__sources  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_members = {
  "baxter_maintenance_msgs__msg",  // message namespace
  "UpdateSources",  // message name
  2,  // number of fields
  sizeof(baxter_maintenance_msgs__msg__UpdateSources),
  baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_member_array,  // message members
  baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_type_support_handle = {
  0,
  &baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_maintenance_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_maintenance_msgs, msg, UpdateSources)() {
  baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_maintenance_msgs, msg, UpdateSource)();
  if (!baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_type_support_handle.typesupport_identifier) {
    baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_maintenance_msgs__msg__UpdateSources__rosidl_typesupport_introspection_c__UpdateSources_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
