// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/AssemblyState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/assembly_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AssemblyState & msg,
  std::ostream & out)
{
  out << "{";
  // member: ready
  {
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << ", ";
  }

  // member: enabled
  {
    out << "enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled, out);
    out << ", ";
  }

  // member: stopped
  {
    out << "stopped: ";
    rosidl_generator_traits::value_to_yaml(msg.stopped, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << ", ";
  }

  // member: estop_button
  {
    out << "estop_button: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_button, out);
    out << ", ";
  }

  // member: estop_source
  {
    out << "estop_source: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_source, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AssemblyState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << "\n";
  }

  // member: enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled, out);
    out << "\n";
  }

  // member: stopped
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stopped: ";
    rosidl_generator_traits::value_to_yaml(msg.stopped, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }

  // member: estop_button
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estop_button: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_button, out);
    out << "\n";
  }

  // member: estop_source
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estop_source: ";
    rosidl_generator_traits::value_to_yaml(msg.estop_source, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AssemblyState & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::AssemblyState & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::AssemblyState & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::AssemblyState>()
{
  return "baxter_core_msgs::msg::AssemblyState";
}

template<>
inline const char * name<baxter_core_msgs::msg::AssemblyState>()
{
  return "baxter_core_msgs/msg/AssemblyState";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::AssemblyState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::AssemblyState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<baxter_core_msgs::msg::AssemblyState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ASSEMBLY_STATE__TRAITS_HPP_
