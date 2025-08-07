// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_data__struct.hpp"
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

void CalibrateArmData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_maintenance_msgs::msg::CalibrateArmData(_init);
}

void CalibrateArmData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_maintenance_msgs::msg::CalibrateArmData *>(message_memory);
  typed_message->~CalibrateArmData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CalibrateArmData_message_member_array[1] = {
  {
    "suppress_write_to_file",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs::msg::CalibrateArmData, suppress_write_to_file),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CalibrateArmData_message_members = {
  "baxter_maintenance_msgs::msg",  // message namespace
  "CalibrateArmData",  // message name
  1,  // number of fields
  sizeof(baxter_maintenance_msgs::msg::CalibrateArmData),
  CalibrateArmData_message_member_array,  // message members
  CalibrateArmData_init_function,  // function to initialize message memory (memory has to be allocated)
  CalibrateArmData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CalibrateArmData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CalibrateArmData_message_members,
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
get_message_type_support_handle<baxter_maintenance_msgs::msg::CalibrateArmData>()
{
  return &::baxter_maintenance_msgs::msg::rosidl_typesupport_introspection_cpp::CalibrateArmData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_maintenance_msgs, msg, CalibrateArmData)() {
  return &::baxter_maintenance_msgs::msg::rosidl_typesupport_introspection_cpp::CalibrateArmData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
