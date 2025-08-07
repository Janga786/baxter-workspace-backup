// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_maintenance_msgs:msg/UpdateSources.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_maintenance_msgs/msg/detail/update_sources__struct.hpp"
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

void UpdateSources_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_maintenance_msgs::msg::UpdateSources(_init);
}

void UpdateSources_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_maintenance_msgs::msg::UpdateSources *>(message_memory);
  typed_message->~UpdateSources();
}

size_t size_function__UpdateSources__sources(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<baxter_maintenance_msgs::msg::UpdateSource> *>(untyped_member);
  return member->size();
}

const void * get_const_function__UpdateSources__sources(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<baxter_maintenance_msgs::msg::UpdateSource> *>(untyped_member);
  return &member[index];
}

void * get_function__UpdateSources__sources(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<baxter_maintenance_msgs::msg::UpdateSource> *>(untyped_member);
  return &member[index];
}

void fetch_function__UpdateSources__sources(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const baxter_maintenance_msgs::msg::UpdateSource *>(
    get_const_function__UpdateSources__sources(untyped_member, index));
  auto & value = *reinterpret_cast<baxter_maintenance_msgs::msg::UpdateSource *>(untyped_value);
  value = item;
}

void assign_function__UpdateSources__sources(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<baxter_maintenance_msgs::msg::UpdateSource *>(
    get_function__UpdateSources__sources(untyped_member, index));
  const auto & value = *reinterpret_cast<const baxter_maintenance_msgs::msg::UpdateSource *>(untyped_value);
  item = value;
}

void resize_function__UpdateSources__sources(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<baxter_maintenance_msgs::msg::UpdateSource> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember UpdateSources_message_member_array[2] = {
  {
    "uuid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs::msg::UpdateSources, uuid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sources",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<baxter_maintenance_msgs::msg::UpdateSource>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_maintenance_msgs::msg::UpdateSources, sources),  // bytes offset in struct
    nullptr,  // default value
    size_function__UpdateSources__sources,  // size() function pointer
    get_const_function__UpdateSources__sources,  // get_const(index) function pointer
    get_function__UpdateSources__sources,  // get(index) function pointer
    fetch_function__UpdateSources__sources,  // fetch(index, &value) function pointer
    assign_function__UpdateSources__sources,  // assign(index, value) function pointer
    resize_function__UpdateSources__sources  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers UpdateSources_message_members = {
  "baxter_maintenance_msgs::msg",  // message namespace
  "UpdateSources",  // message name
  2,  // number of fields
  sizeof(baxter_maintenance_msgs::msg::UpdateSources),
  UpdateSources_message_member_array,  // message members
  UpdateSources_init_function,  // function to initialize message memory (memory has to be allocated)
  UpdateSources_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t UpdateSources_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &UpdateSources_message_members,
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
get_message_type_support_handle<baxter_maintenance_msgs::msg::UpdateSources>()
{
  return &::baxter_maintenance_msgs::msg::rosidl_typesupport_introspection_cpp::UpdateSources_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_maintenance_msgs, msg, UpdateSources)() {
  return &::baxter_maintenance_msgs::msg::rosidl_typesupport_introspection_cpp::UpdateSources_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
