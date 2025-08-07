// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_core_msgs:msg/DigitalIOStates.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_core_msgs/msg/detail/digital_io_states__struct.hpp"
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

void DigitalIOStates_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_core_msgs::msg::DigitalIOStates(_init);
}

void DigitalIOStates_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_core_msgs::msg::DigitalIOStates *>(message_memory);
  typed_message->~DigitalIOStates();
}

size_t size_function__DigitalIOStates__names(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DigitalIOStates__names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__DigitalIOStates__names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__DigitalIOStates__names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__DigitalIOStates__names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__DigitalIOStates__names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__DigitalIOStates__names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__DigitalIOStates__names(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__DigitalIOStates__states(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<baxter_core_msgs::msg::DigitalIOState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__DigitalIOStates__states(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<baxter_core_msgs::msg::DigitalIOState> *>(untyped_member);
  return &member[index];
}

void * get_function__DigitalIOStates__states(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<baxter_core_msgs::msg::DigitalIOState> *>(untyped_member);
  return &member[index];
}

void fetch_function__DigitalIOStates__states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const baxter_core_msgs::msg::DigitalIOState *>(
    get_const_function__DigitalIOStates__states(untyped_member, index));
  auto & value = *reinterpret_cast<baxter_core_msgs::msg::DigitalIOState *>(untyped_value);
  value = item;
}

void assign_function__DigitalIOStates__states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<baxter_core_msgs::msg::DigitalIOState *>(
    get_function__DigitalIOStates__states(untyped_member, index));
  const auto & value = *reinterpret_cast<const baxter_core_msgs::msg::DigitalIOState *>(untyped_value);
  item = value;
}

void resize_function__DigitalIOStates__states(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<baxter_core_msgs::msg::DigitalIOState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember DigitalIOStates_message_member_array[2] = {
  {
    "names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::DigitalIOStates, names),  // bytes offset in struct
    nullptr,  // default value
    size_function__DigitalIOStates__names,  // size() function pointer
    get_const_function__DigitalIOStates__names,  // get_const(index) function pointer
    get_function__DigitalIOStates__names,  // get(index) function pointer
    fetch_function__DigitalIOStates__names,  // fetch(index, &value) function pointer
    assign_function__DigitalIOStates__names,  // assign(index, value) function pointer
    resize_function__DigitalIOStates__names  // resize(index) function pointer
  },
  {
    "states",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<baxter_core_msgs::msg::DigitalIOState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::DigitalIOStates, states),  // bytes offset in struct
    nullptr,  // default value
    size_function__DigitalIOStates__states,  // size() function pointer
    get_const_function__DigitalIOStates__states,  // get_const(index) function pointer
    get_function__DigitalIOStates__states,  // get(index) function pointer
    fetch_function__DigitalIOStates__states,  // fetch(index, &value) function pointer
    assign_function__DigitalIOStates__states,  // assign(index, value) function pointer
    resize_function__DigitalIOStates__states  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers DigitalIOStates_message_members = {
  "baxter_core_msgs::msg",  // message namespace
  "DigitalIOStates",  // message name
  2,  // number of fields
  sizeof(baxter_core_msgs::msg::DigitalIOStates),
  DigitalIOStates_message_member_array,  // message members
  DigitalIOStates_init_function,  // function to initialize message memory (memory has to be allocated)
  DigitalIOStates_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t DigitalIOStates_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &DigitalIOStates_message_members,
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
get_message_type_support_handle<baxter_core_msgs::msg::DigitalIOStates>()
{
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::DigitalIOStates_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, msg, DigitalIOStates)() {
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::DigitalIOStates_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
