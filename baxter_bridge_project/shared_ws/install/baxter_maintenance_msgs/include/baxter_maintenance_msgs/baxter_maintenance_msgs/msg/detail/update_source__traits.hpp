// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateSource.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__TRAITS_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_maintenance_msgs/msg/detail/update_source__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_maintenance_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UpdateSource & msg,
  std::ostream & out)
{
  out << "{";
  // member: devname
  {
    out << "devname: ";
    rosidl_generator_traits::value_to_yaml(msg.devname, out);
    out << ", ";
  }

  // member: filename
  {
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << ", ";
  }

  // member: version
  {
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << ", ";
  }

  // member: uuid
  {
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateSource & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: devname
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "devname: ";
    rosidl_generator_traits::value_to_yaml(msg.devname, out);
    out << "\n";
  }

  // member: filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "filename: ";
    rosidl_generator_traits::value_to_yaml(msg.filename, out);
    out << "\n";
  }

  // member: version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "version: ";
    rosidl_generator_traits::value_to_yaml(msg.version, out);
    out << "\n";
  }

  // member: uuid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateSource & msg, bool use_flow_style = false)
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
  const baxter_maintenance_msgs::msg::UpdateSource & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_maintenance_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_maintenance_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_maintenance_msgs::msg::UpdateSource & msg)
{
  return baxter_maintenance_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_maintenance_msgs::msg::UpdateSource>()
{
  return "baxter_maintenance_msgs::msg::UpdateSource";
}

template<>
inline const char * name<baxter_maintenance_msgs::msg::UpdateSource>()
{
  return "baxter_maintenance_msgs/msg/UpdateSource";
}

template<>
struct has_fixed_size<baxter_maintenance_msgs::msg::UpdateSource>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_maintenance_msgs::msg::UpdateSource>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_maintenance_msgs::msg::UpdateSource>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__TRAITS_HPP_
