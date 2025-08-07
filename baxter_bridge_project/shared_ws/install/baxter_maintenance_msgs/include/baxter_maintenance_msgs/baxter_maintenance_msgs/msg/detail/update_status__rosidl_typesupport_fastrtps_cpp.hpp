// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "baxter_maintenance_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "baxter_maintenance_msgs/msg/detail/update_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace baxter_maintenance_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_baxter_maintenance_msgs
cdr_serialize(
  const baxter_maintenance_msgs::msg::UpdateStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_baxter_maintenance_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  baxter_maintenance_msgs::msg::UpdateStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_baxter_maintenance_msgs
get_serialized_size(
  const baxter_maintenance_msgs::msg::UpdateStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_baxter_maintenance_msgs
max_serialized_size_UpdateStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace baxter_maintenance_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_baxter_maintenance_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, baxter_maintenance_msgs, msg, UpdateStatus)();

#ifdef __cplusplus
}
#endif

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
