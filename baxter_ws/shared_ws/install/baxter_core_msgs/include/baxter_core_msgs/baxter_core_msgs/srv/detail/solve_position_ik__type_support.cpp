// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from baxter_core_msgs:srv/SolvePositionIK.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_core_msgs/srv/detail/solve_position_ik__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace baxter_core_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SolvePositionIK_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_core_msgs::srv::SolvePositionIK_Request(_init);
}

void SolvePositionIK_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_core_msgs::srv::SolvePositionIK_Request *>(message_memory);
  typed_message->~SolvePositionIK_Request();
}

size_t size_function__SolvePositionIK_Request__pose_stamp(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SolvePositionIK_Request__pose_stamp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void * get_function__SolvePositionIK_Request__pose_stamp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void fetch_function__SolvePositionIK_Request__pose_stamp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__SolvePositionIK_Request__pose_stamp(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__SolvePositionIK_Request__pose_stamp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__SolvePositionIK_Request__pose_stamp(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

void resize_function__SolvePositionIK_Request__pose_stamp(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SolvePositionIK_Request__seed_angles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SolvePositionIK_Request__seed_angles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return &member[index];
}

void * get_function__SolvePositionIK_Request__seed_angles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return &member[index];
}

void fetch_function__SolvePositionIK_Request__seed_angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const sensor_msgs::msg::JointState *>(
    get_const_function__SolvePositionIK_Request__seed_angles(untyped_member, index));
  auto & value = *reinterpret_cast<sensor_msgs::msg::JointState *>(untyped_value);
  value = item;
}

void assign_function__SolvePositionIK_Request__seed_angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<sensor_msgs::msg::JointState *>(
    get_function__SolvePositionIK_Request__seed_angles(untyped_member, index));
  const auto & value = *reinterpret_cast<const sensor_msgs::msg::JointState *>(untyped_value);
  item = value;
}

void resize_function__SolvePositionIK_Request__seed_angles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SolvePositionIK_Request_message_member_array[3] = {
  {
    "pose_stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::srv::SolvePositionIK_Request, pose_stamp),  // bytes offset in struct
    nullptr,  // default value
    size_function__SolvePositionIK_Request__pose_stamp,  // size() function pointer
    get_const_function__SolvePositionIK_Request__pose_stamp,  // get_const(index) function pointer
    get_function__SolvePositionIK_Request__pose_stamp,  // get(index) function pointer
    fetch_function__SolvePositionIK_Request__pose_stamp,  // fetch(index, &value) function pointer
    assign_function__SolvePositionIK_Request__pose_stamp,  // assign(index, value) function pointer
    resize_function__SolvePositionIK_Request__pose_stamp  // resize(index) function pointer
  },
  {
    "seed_angles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::JointState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::srv::SolvePositionIK_Request, seed_angles),  // bytes offset in struct
    nullptr,  // default value
    size_function__SolvePositionIK_Request__seed_angles,  // size() function pointer
    get_const_function__SolvePositionIK_Request__seed_angles,  // get_const(index) function pointer
    get_function__SolvePositionIK_Request__seed_angles,  // get(index) function pointer
    fetch_function__SolvePositionIK_Request__seed_angles,  // fetch(index, &value) function pointer
    assign_function__SolvePositionIK_Request__seed_angles,  // assign(index, value) function pointer
    resize_function__SolvePositionIK_Request__seed_angles  // resize(index) function pointer
  },
  {
    "seed_mode",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::srv::SolvePositionIK_Request, seed_mode),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SolvePositionIK_Request_message_members = {
  "baxter_core_msgs::srv",  // message namespace
  "SolvePositionIK_Request",  // message name
  3,  // number of fields
  sizeof(baxter_core_msgs::srv::SolvePositionIK_Request),
  SolvePositionIK_Request_message_member_array,  // message members
  SolvePositionIK_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SolvePositionIK_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SolvePositionIK_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SolvePositionIK_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace baxter_core_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<baxter_core_msgs::srv::SolvePositionIK_Request>()
{
  return &::baxter_core_msgs::srv::rosidl_typesupport_introspection_cpp::SolvePositionIK_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, srv, SolvePositionIK_Request)() {
  return &::baxter_core_msgs::srv::rosidl_typesupport_introspection_cpp::SolvePositionIK_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "baxter_core_msgs/srv/detail/solve_position_ik__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace baxter_core_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SolvePositionIK_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) baxter_core_msgs::srv::SolvePositionIK_Response(_init);
}

void SolvePositionIK_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<baxter_core_msgs::srv::SolvePositionIK_Response *>(message_memory);
  typed_message->~SolvePositionIK_Response();
}

size_t size_function__SolvePositionIK_Response__joints(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SolvePositionIK_Response__joints(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return &member[index];
}

void * get_function__SolvePositionIK_Response__joints(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  return &member[index];
}

void fetch_function__SolvePositionIK_Response__joints(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const sensor_msgs::msg::JointState *>(
    get_const_function__SolvePositionIK_Response__joints(untyped_member, index));
  auto & value = *reinterpret_cast<sensor_msgs::msg::JointState *>(untyped_value);
  value = item;
}

void assign_function__SolvePositionIK_Response__joints(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<sensor_msgs::msg::JointState *>(
    get_function__SolvePositionIK_Response__joints(untyped_member, index));
  const auto & value = *reinterpret_cast<const sensor_msgs::msg::JointState *>(untyped_value);
  item = value;
}

void resize_function__SolvePositionIK_Response__joints(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<sensor_msgs::msg::JointState> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SolvePositionIK_Response__is_valid(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__SolvePositionIK_Response__is_valid(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__SolvePositionIK_Response__is_valid(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__SolvePositionIK_Response__is_valid(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SolvePositionIK_Response__result_type(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SolvePositionIK_Response__result_type(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SolvePositionIK_Response__result_type(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SolvePositionIK_Response__result_type(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__SolvePositionIK_Response__result_type(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__SolvePositionIK_Response__result_type(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__SolvePositionIK_Response__result_type(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__SolvePositionIK_Response__result_type(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SolvePositionIK_Response_message_member_array[3] = {
  {
    "joints",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<sensor_msgs::msg::JointState>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::srv::SolvePositionIK_Response, joints),  // bytes offset in struct
    nullptr,  // default value
    size_function__SolvePositionIK_Response__joints,  // size() function pointer
    get_const_function__SolvePositionIK_Response__joints,  // get_const(index) function pointer
    get_function__SolvePositionIK_Response__joints,  // get(index) function pointer
    fetch_function__SolvePositionIK_Response__joints,  // fetch(index, &value) function pointer
    assign_function__SolvePositionIK_Response__joints,  // assign(index, value) function pointer
    resize_function__SolvePositionIK_Response__joints  // resize(index) function pointer
  },
  {
    "is_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::srv::SolvePositionIK_Response, is_valid),  // bytes offset in struct
    nullptr,  // default value
    size_function__SolvePositionIK_Response__is_valid,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__SolvePositionIK_Response__is_valid,  // fetch(index, &value) function pointer
    assign_function__SolvePositionIK_Response__is_valid,  // assign(index, value) function pointer
    resize_function__SolvePositionIK_Response__is_valid  // resize(index) function pointer
  },
  {
    "result_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(baxter_core_msgs::srv::SolvePositionIK_Response, result_type),  // bytes offset in struct
    nullptr,  // default value
    size_function__SolvePositionIK_Response__result_type,  // size() function pointer
    get_const_function__SolvePositionIK_Response__result_type,  // get_const(index) function pointer
    get_function__SolvePositionIK_Response__result_type,  // get(index) function pointer
    fetch_function__SolvePositionIK_Response__result_type,  // fetch(index, &value) function pointer
    assign_function__SolvePositionIK_Response__result_type,  // assign(index, value) function pointer
    resize_function__SolvePositionIK_Response__result_type  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SolvePositionIK_Response_message_members = {
  "baxter_core_msgs::srv",  // message namespace
  "SolvePositionIK_Response",  // message name
  3,  // number of fields
  sizeof(baxter_core_msgs::srv::SolvePositionIK_Response),
  SolvePositionIK_Response_message_member_array,  // message members
  SolvePositionIK_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SolvePositionIK_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SolvePositionIK_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SolvePositionIK_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace baxter_core_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<baxter_core_msgs::srv::SolvePositionIK_Response>()
{
  return &::baxter_core_msgs::srv::rosidl_typesupport_introspection_cpp::SolvePositionIK_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, srv, SolvePositionIK_Response)() {
  return &::baxter_core_msgs::srv::rosidl_typesupport_introspection_cpp::SolvePositionIK_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "baxter_core_msgs/srv/detail/solve_position_ik__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace baxter_core_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SolvePositionIK_service_members = {
  "baxter_core_msgs::srv",  // service namespace
  "SolvePositionIK",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<baxter_core_msgs::srv::SolvePositionIK>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SolvePositionIK_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SolvePositionIK_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace baxter_core_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<baxter_core_msgs::srv::SolvePositionIK>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::baxter_core_msgs::srv::rosidl_typesupport_introspection_cpp::SolvePositionIK_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::baxter_core_msgs::srv::SolvePositionIK_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::baxter_core_msgs::srv::SolvePositionIK_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, baxter_core_msgs, srv, SolvePositionIK)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<baxter_core_msgs::srv::SolvePositionIK>();
}

#ifdef __cplusplus
}
#endif
