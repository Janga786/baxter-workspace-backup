// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from baxter_maintenance_msgs:msg/TareEnable.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "baxter_maintenance_msgs/msg/detail/tare_enable__rosidl_typesupport_introspection_c.h"
#include "baxter_maintenance_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "baxter_maintenance_msgs/msg/detail/tare_enable__functions.h"
#include "baxter_maintenance_msgs/msg/detail/tare_enable__struct.h"


// Include directives for member types
// Member `uid`
#include "rosidl_runtime_c/string_functions.h"
// Member `data`
#include "baxter_maintenance_msgs/msg/tare_data.h"
// Member `data`
#include "baxter_maintenance_msgs/msg/detail/tare_data__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  baxter_maintenance_msgs__msg__TareEnable__init(message_memory);
}

void baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_fini_function(void * message_memory)
{
  baxter_maintenance_msgs__msg__TareEnable__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_member_array[3] = {
  {
    "is_enabled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs__msg__TareEnable, is_enabled),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs__msg__TareEnable, uid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs__msg__TareEnable, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_members = {
  "baxter_maintenance_msgs__msg",  // message namespace
  "TareEnable",  // message name
  3,  // number of fields
  sizeof(baxter_maintenance_msgs__msg__TareEnable),
  baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_member_array,  // message members
  baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_init_function,  // function to initialize message memory (memory has to be allocated)
  baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_type_support_handle = {
  0,
  &baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_baxter_maintenance_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_maintenance_msgs, msg, TareEnable)() {
  baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, baxter_maintenance_msgs, msg, TareData)();
  if (!baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_type_support_handle.typesupport_identifier) {
    baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &baxter_maintenance_msgs__msg__TareEnable__rosidl_typesupport_introspection_c__TareEnable_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
