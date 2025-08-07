// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_core_msgs:msg/SEAJointState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_core_msgs/msg/detail/sea_joint_state__struct.hpp"
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

void SEAJointState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_core_msgs::msg::SEAJointState(_init);
}

void SEAJointState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_core_msgs::msg::SEAJointState *>(message_memory);
  typed_message->~SEAJointState();
}

size_t size_function__SEAJointState__name(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__name(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__name(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__name(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__SEAJointState__name(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__name(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__SEAJointState__name(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__name(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__commanded_position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__commanded_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__commanded_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__commanded_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__commanded_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__commanded_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__commanded_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__commanded_position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__commanded_velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__commanded_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__commanded_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__commanded_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__commanded_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__commanded_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__commanded_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__commanded_velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__commanded_acceleration(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__commanded_acceleration(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__commanded_acceleration(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__commanded_acceleration(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__commanded_acceleration(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__commanded_acceleration(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__commanded_acceleration(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__commanded_acceleration(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__commanded_effort(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__commanded_effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__commanded_effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__commanded_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__commanded_effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__commanded_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__commanded_effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__commanded_effort(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__actual_position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__actual_position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__actual_position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__actual_position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__actual_position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__actual_position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__actual_position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__actual_position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__actual_velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__actual_velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__actual_velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__actual_velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__actual_velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__actual_velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__actual_velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__actual_velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__actual_effort(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__actual_effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__actual_effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__actual_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__actual_effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__actual_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__actual_effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__actual_effort(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__gravity_model_effort(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__gravity_model_effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__gravity_model_effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__gravity_model_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__gravity_model_effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__gravity_model_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__gravity_model_effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__gravity_model_effort(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__gravity_only(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__gravity_only(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__gravity_only(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__gravity_only(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__gravity_only(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__gravity_only(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__gravity_only(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__gravity_only(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__hysteresis_model_effort(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__hysteresis_model_effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__hysteresis_model_effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__hysteresis_model_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__hysteresis_model_effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__hysteresis_model_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__hysteresis_model_effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__hysteresis_model_effort(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SEAJointState__crosstalk_model_effort(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SEAJointState__crosstalk_model_effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__SEAJointState__crosstalk_model_effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__SEAJointState__crosstalk_model_effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SEAJointState__crosstalk_model_effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SEAJointState__crosstalk_model_effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SEAJointState__crosstalk_model_effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__SEAJointState__crosstalk_model_effort(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SEAJointState_message_member_array[14] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, name),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__name,  // size() function pointer
    get_const_function__SEAJointState__name,  // get_const(index) function pointer
    get_function__SEAJointState__name,  // get(index) function pointer
    fetch_function__SEAJointState__name,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__name,  // assign(index, value) function pointer
    resize_function__SEAJointState__name  // resize(index) function pointer
  },
  {
    "commanded_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, commanded_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__commanded_position,  // size() function pointer
    get_const_function__SEAJointState__commanded_position,  // get_const(index) function pointer
    get_function__SEAJointState__commanded_position,  // get(index) function pointer
    fetch_function__SEAJointState__commanded_position,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__commanded_position,  // assign(index, value) function pointer
    resize_function__SEAJointState__commanded_position  // resize(index) function pointer
  },
  {
    "commanded_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, commanded_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__commanded_velocity,  // size() function pointer
    get_const_function__SEAJointState__commanded_velocity,  // get_const(index) function pointer
    get_function__SEAJointState__commanded_velocity,  // get(index) function pointer
    fetch_function__SEAJointState__commanded_velocity,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__commanded_velocity,  // assign(index, value) function pointer
    resize_function__SEAJointState__commanded_velocity  // resize(index) function pointer
  },
  {
    "commanded_acceleration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, commanded_acceleration),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__commanded_acceleration,  // size() function pointer
    get_const_function__SEAJointState__commanded_acceleration,  // get_const(index) function pointer
    get_function__SEAJointState__commanded_acceleration,  // get(index) function pointer
    fetch_function__SEAJointState__commanded_acceleration,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__commanded_acceleration,  // assign(index, value) function pointer
    resize_function__SEAJointState__commanded_acceleration  // resize(index) function pointer
  },
  {
    "commanded_effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, commanded_effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__commanded_effort,  // size() function pointer
    get_const_function__SEAJointState__commanded_effort,  // get_const(index) function pointer
    get_function__SEAJointState__commanded_effort,  // get(index) function pointer
    fetch_function__SEAJointState__commanded_effort,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__commanded_effort,  // assign(index, value) function pointer
    resize_function__SEAJointState__commanded_effort  // resize(index) function pointer
  },
  {
    "actual_position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, actual_position),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__actual_position,  // size() function pointer
    get_const_function__SEAJointState__actual_position,  // get_const(index) function pointer
    get_function__SEAJointState__actual_position,  // get(index) function pointer
    fetch_function__SEAJointState__actual_position,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__actual_position,  // assign(index, value) function pointer
    resize_function__SEAJointState__actual_position  // resize(index) function pointer
  },
  {
    "actual_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, actual_velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__actual_velocity,  // size() function pointer
    get_const_function__SEAJointState__actual_velocity,  // get_const(index) function pointer
    get_function__SEAJointState__actual_velocity,  // get(index) function pointer
    fetch_function__SEAJointState__actual_velocity,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__actual_velocity,  // assign(index, value) function pointer
    resize_function__SEAJointState__actual_velocity  // resize(index) function pointer
  },
  {
    "actual_effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, actual_effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__actual_effort,  // size() function pointer
    get_const_function__SEAJointState__actual_effort,  // get_const(index) function pointer
    get_function__SEAJointState__actual_effort,  // get(index) function pointer
    fetch_function__SEAJointState__actual_effort,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__actual_effort,  // assign(index, value) function pointer
    resize_function__SEAJointState__actual_effort  // resize(index) function pointer
  },
  {
    "gravity_model_effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, gravity_model_effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__gravity_model_effort,  // size() function pointer
    get_const_function__SEAJointState__gravity_model_effort,  // get_const(index) function pointer
    get_function__SEAJointState__gravity_model_effort,  // get(index) function pointer
    fetch_function__SEAJointState__gravity_model_effort,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__gravity_model_effort,  // assign(index, value) function pointer
    resize_function__SEAJointState__gravity_model_effort  // resize(index) function pointer
  },
  {
    "gravity_only",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, gravity_only),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__gravity_only,  // size() function pointer
    get_const_function__SEAJointState__gravity_only,  // get_const(index) function pointer
    get_function__SEAJointState__gravity_only,  // get(index) function pointer
    fetch_function__SEAJointState__gravity_only,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__gravity_only,  // assign(index, value) function pointer
    resize_function__SEAJointState__gravity_only  // resize(index) function pointer
  },
  {
    "hysteresis_model_effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, hysteresis_model_effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__hysteresis_model_effort,  // size() function pointer
    get_const_function__SEAJointState__hysteresis_model_effort,  // get_const(index) function pointer
    get_function__SEAJointState__hysteresis_model_effort,  // get(index) function pointer
    fetch_function__SEAJointState__hysteresis_model_effort,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__hysteresis_model_effort,  // assign(index, value) function pointer
    resize_function__SEAJointState__hysteresis_model_effort  // resize(index) function pointer
  },
  {
    "crosstalk_model_effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, crosstalk_model_effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__SEAJointState__crosstalk_model_effort,  // size() function pointer
    get_const_function__SEAJointState__crosstalk_model_effort,  // get_const(index) function pointer
    get_function__SEAJointState__crosstalk_model_effort,  // get(index) function pointer
    fetch_function__SEAJointState__crosstalk_model_effort,  // fetch(index, &value) function pointer
    assign_function__SEAJointState__crosstalk_model_effort,  // assign(index, value) function pointer
    resize_function__SEAJointState__crosstalk_model_effort  // resize(index) function pointer
  },
  {
    "hyst_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::msg::SEAJointState, hyst_state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SEAJointState_message_members = {
  "baxter_core_msgs::msg",  // message namespace
  "SEAJointState",  // message name
  14,  // number of fields
  sizeof(baxter_core_msgs::msg::SEAJointState),
  SEAJointState_message_member_array,  // message members
  SEAJointState_init_function,  // function to initialize message memory (memory has to be allocated)
  SEAJointState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SEAJointState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SEAJointState_message_members,
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
get_message_type_support_handle<baxter_core_msgs::msg::SEAJointState>()
{
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::SEAJointState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, msg, SEAJointState)() {
  return &::baxter_core_msgs::msg::rosidl_typesupport_introspection_cpp::SEAJointState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
