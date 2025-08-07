// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_core_msgs:msg/NavigatorState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_core_msgs/msg/detail/navigator_state__struct.hpp"
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

void NavigatorState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_core_msgs::msg::NavigatorState(_init);
}

void NavigatorState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_core_msgs::msg::NavigatorState *>(message_memory);
  typed_message->~NavigatorState();
}

size_t size_function__NavigatorState__button_names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NavigatorState__button_names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__NavigatorState__button_names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__NavigatorState__button_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__NavigatorState__button_names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__NavigatorState__button_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__NavigatorState__button_names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__NavigatorState__button_names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__NavigatorState__buttons(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__NavigatorState__buttons(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__NavigatorState__buttons(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__NavigatorState__buttons(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__NavigatorState__light_names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__NavigatorState__light_names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__NavigatorState__light_names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__NavigatorState__light_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__NavigatorState__light_names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__NavigatorState__light_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__NavigatorState__light_names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__NavigatorState__light_names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__NavigatorState__lights(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__NavigatorState__lights(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__NavigatorState__lights(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__NavigatorState__lights(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NavigatorState_message_member_array[5] = {
  {
    "button_names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::NavigatorState, button_names),  // bytes offset in struct
    nullptr,  // default value
    size_function__NavigatorState__button_names,  // size() function pointer
    get_const_function__NavigatorState__button_names,  // get_const(index) function pointer
    get_function__NavigatorState__button_names,  // get(index) function pointer
    fetch_function__NavigatorState__button_names,  // fetch(index, &value) function pointer
    assign_function__NavigatorState__button_names,  // assign(index, value) function pointer
    resize_function__NavigatorState__button_names  // resize(index) function pointer
  },
  {
    "buttons",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::NavigatorState, buttons),  // bytes offset in struct
    nullptr,  // default value
    size_function__NavigatorState__buttons,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__NavigatorState__buttons,  // fetch(index, &value) function pointer
    assign_function__NavigatorState__buttons,  // assign(index, value) function pointer
    resize_function__NavigatorState__buttons  // resize(index) function pointer
  },
  {
    "wheel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::NavigatorState, wheel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "light_names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::NavigatorState, light_names),  // bytes offset in struct
    nullptr,  // default value
    size_function__NavigatorState__light_names,  // size() function pointer
    get_const_function__NavigatorState__light_names,  // get_const(index) function pointer
    get_function__NavigatorState__light_names,  // get(index) function pointer
    fetch_function__NavigatorState__light_names,  // fetch(index, &value) function pointer
    assign_function__NavigatorState__light_names,  // assign(index, value) function pointer
    resize_function__NavigatorState__light_names  // resize(index) function pointer
  },
  {
    "lights",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::NavigatorState, lights),  // bytes offset in struct
    nullptr,  // default value
    size_function__NavigatorState__lights,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__NavigatorState__lights,  // fetch(index, &value) function pointer
    assign_function__NavigatorState__lights,  // assign(index, value) function pointer
    resize_function__NavigatorState__lights  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NavigatorState_message_members = {
  "baxter_core_msgs::msg",  // message namespace
  "NavigatorState",  // message name
  5,  // number of fields
  sizeof(baxter_core_msgs::msg::NavigatorState),
  NavigatorState_message_member_array,  // message members
  NavigatorState_init_function,  // function to initialize message memory (memory has to be allocated)
  NavigatorState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NavigatorState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NavigatorState_message_members,
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
get_message_type_support_handle<baxter_core_msgs::msg::NavigatorState>()
{
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::NavigatorState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, msg, NavigatorState)() {
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::NavigatorState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
