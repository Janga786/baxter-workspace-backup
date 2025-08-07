// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__TRAITS_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_maintenance_msgs/msg/detail/update_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_maintenance_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UpdateStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: progress
  {
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << ", ";
  }

  // member: long_description
  {
    out << "long_description: ";
    rosidl_generator_traits::value_to_yaml(msg.long_description, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "progress: ";
    rosidl_generator_traits::value_to_yaml(msg.progress, out);
    out << "\n";
  }

  // member: long_description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "long_description: ";
    rosidl_generator_traits::value_to_yaml(msg.long_description, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace baxter_maintenance_msgs

namespace rosidl_generator_traits
{

[[deprecated("use baxter_maintenance_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const baxter_maintenance_msgs::msg::UpdateStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_maintenance_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_maintenance_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_maintenance_msgs::msg::UpdateStatus & msg)
{
  return baxter_maintenance_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_maintenance_msgs::msg::UpdateStatus>()
{
  return "baxter_maintenance_msgs::msg::UpdateStatus";
}

template<>
inline const char * name<baxter_maintenance_msgs::msg::UpdateStatus>()
{
  return "baxter_maintenance_msgs/msg/UpdateStatus";
}

template<>
struct has_fixed_size<baxter_maintenance_msgs::msg::UpdateStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_maintenance_msgs::msg::UpdateStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_maintenance_msgs::msg::UpdateStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__TRAITS_HPP_
