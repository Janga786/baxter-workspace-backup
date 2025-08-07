// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/URDFConfiguration.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/urdf_configuration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const URDFConfiguration & msg,
  std::ostream & out)
{
  out << "{";
  // member: time
  {
    out << "time: ";
    to_flow_style_yaml(msg.time, out);
    out << ", ";
  }

  // member: link
  {
    out << "link: ";
    rosidl_generator_traits::value_to_yaml(msg.link, out);
    out << ", ";
  }

  // member: joint
  {
    out << "joint: ";
    rosidl_generator_traits::value_to_yaml(msg.joint, out);
    out << ", ";
  }

  // member: urdf
  {
    out << "urdf: ";
    rosidl_generator_traits::value_to_yaml(msg.urdf, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const URDFConfiguration & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time:\n";
    to_block_style_yaml(msg.time, out, indentation + 2);
  }

  // member: link
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "link: ";
    rosidl_generator_traits::value_to_yaml(msg.link, out);
    out << "\n";
  }

  // member: joint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint: ";
    rosidl_generator_traits::value_to_yaml(msg.joint, out);
    out << "\n";
  }

  // member: urdf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "urdf: ";
    rosidl_generator_traits::value_to_yaml(msg.urdf, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const URDFConfiguration & msg, bool use_flow_style = false)
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

}  // namespace baxter_core_msgs

namespace rosidl_generator_traits
{

[[deprecated("use baxter_core_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const baxter_core_msgs::msg::URDFConfiguration & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::URDFConfiguration & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::URDFConfiguration>()
{
  return "baxter_core_msgs::msg::URDFConfiguration";
}

template<>
inline const char * name<baxter_core_msgs::msg::URDFConfiguration>()
{
  return "baxter_core_msgs/msg/URDFConfiguration";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::URDFConfiguration>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::URDFConfiguration>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::URDFConfiguration>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__URDF_CONFIGURATION__TRAITS_HPP_
