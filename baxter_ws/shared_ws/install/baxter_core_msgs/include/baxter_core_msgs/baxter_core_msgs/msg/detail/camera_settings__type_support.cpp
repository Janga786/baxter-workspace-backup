// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_core_msgs/msg/detail/camera_settings__struct.hpp"
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

void CameraSettings_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_core_msgs::msg::CameraSettings(_init);
}

void CameraSettings_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_core_msgs::msg::CameraSettings *>(message_memory);
  typed_message->~CameraSettings();
}

size_t size_function__CameraSettings__controls(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<baxter_core_msgs::msg::CameraControl> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CameraSettings__controls(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<baxter_core_msgs::msg::CameraControl> *>(untyped_member);
  return &member[index];
}

void * get_function__CameraSettings__controls(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<baxter_core_msgs::msg::CameraControl> *>(untyped_member);
  return &member[index];
}

void fetch_function__CameraSettings__controls(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const baxter_core_msgs::msg::CameraControl *>(
    get_const_function__CameraSettings__controls(untyped_member, index));
  auto & value = *reinterpret_cast<baxter_core_msgs::msg::CameraControl *>(untyped_value);
  value = item;
}

void assign_function__CameraSettings__controls(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<baxter_core_msgs::msg::CameraControl *>(
    get_function__CameraSettings__controls(untyped_member, index));
  const auto & value = *reinterpret_cast<const baxter_core_msgs::msg::CameraControl *>(untyped_value);
  item = value;
}

void resize_function__CameraSettings__controls(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<baxter_core_msgs::msg::CameraControl> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CameraSettings_message_member_array[4] = {
  {
    "width",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::CameraSettings, width),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "height",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::CameraSettings, height),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::CameraSettings, fps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "controls",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<baxter_core_msgs::msg::CameraControl>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::CameraSettings, controls),  // bytes offset in struct
    nullptr,  // default value
    size_function__CameraSettings__controls,  // size() function pointer
    get_const_function__CameraSettings__controls,  // get_const(index) function pointer
    get_function__CameraSettings__controls,  // get(index) function pointer
    fetch_function__CameraSettings__controls,  // fetch(index, &value) function pointer
    assign_function__CameraSettings__controls,  // assign(index, value) function pointer
    resize_function__CameraSettings__controls  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CameraSettings_message_members = {
  "baxter_core_msgs::msg",  // message namespace
  "CameraSettings",  // message name
  4,  // number of fields
  sizeof(baxter_core_msgs::msg::CameraSettings),
  CameraSettings_message_member_array,  // message members
  CameraSettings_init_function,  // function to initialize message memory (memory has to be allocated)
  CameraSettings_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CameraSettings_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CameraSettings_message_members,
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
get_message_type_support_handle<baxter_core_msgs::msg::CameraSettings>()
{
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::CameraSettings_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, msg, CameraSettings)() {
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::CameraSettings_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
