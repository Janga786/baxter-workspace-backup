// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_core_msgs:msg/CameraControl.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_core_msgs/msg/detail/camera_control__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace baxter_core_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CameraControl_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_core_msgs::msg::CameraControl(_init);
}

void CameraControl_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_core_msgs::msg::CameraControl *>(message_memory);
  typed_message->~CameraControl();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CameraControl_message_member_array[2] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::CameraControl, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "value",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::CameraControl, value),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CameraControl_message_members = {
  "baxter_core_msgs::msg",  // message namespace
  "CameraControl",  // message name
  2,  // number of fields
  sizeof(baxter_core_msgs::msg::CameraControl),
  CameraControl_message_member_array,  // message members
  CameraControl_init_function,  // function to initialize message memory (memory has to be allocated)
  CameraControl_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CameraControl_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CameraControl_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace baxter_core_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<baxter_core_msgs::msg::CameraControl>()
{
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::CameraControl_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, msg, CameraControl)() {
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::CameraControl_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
