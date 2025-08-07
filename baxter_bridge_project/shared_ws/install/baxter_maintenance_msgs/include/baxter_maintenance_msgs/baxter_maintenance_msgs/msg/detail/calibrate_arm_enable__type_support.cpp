// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmEnable.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_enable__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace baxter_maintenance_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CalibrateArmEnable_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_maintenance_msgs::msg::CalibrateArmEnable(_init);
}

void CalibrateArmEnable_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_maintenance_msgs::msg::CalibrateArmEnable *>(message_memory);
  typed_message->~CalibrateArmEnable();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CalibrateArmEnable_message_member_array[3] = {
  {
    "is_enabled",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs::msg::CalibrateArmEnable, is_enabled),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "uid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs::msg::CalibrateArmEnable, uid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<baxter_maintenance_msgs::msg::CalibrateArmData>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs::msg::CalibrateArmEnable, data),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CalibrateArmEnable_message_members = {
  "baxter_maintenance_msgs::msg",  // message namespace
  "CalibrateArmEnable",  // message name
  3,  // number of fields
  sizeof(baxter_maintenance_msgs::msg::CalibrateArmEnable),
  CalibrateArmEnable_message_member_array,  // message members
  CalibrateArmEnable_init_function,  // function to initialize message memory (memory has to be allocated)
  CalibrateArmEnable_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CalibrateArmEnable_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CalibrateArmEnable_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace baxter_maintenance_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<baxter_maintenance_msgs::msg::CalibrateArmEnable>()
{
  return &::baxter_maintenance_msgs::msg::rosidl_typesupport_introspection_cpp::CalibrateArmEnable_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_maintenance_msgs, msg, CalibrateArmEnable)() {
  return &::baxter_maintenance_msgs::msg::rosidl_typesupport_introspection_cpp::CalibrateArmEnable_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
