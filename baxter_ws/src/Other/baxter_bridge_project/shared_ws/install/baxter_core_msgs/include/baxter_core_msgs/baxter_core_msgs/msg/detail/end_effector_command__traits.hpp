// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/end_effector_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EndEffectorCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: args
  {
    out << "args: ";
    rosidl_generator_traits::value_to_yaml(msg.args, out);
    out << ", ";
  }

  // member: sender
  {
    out << "sender: ";
    rosidl_generator_traits::value_to_yaml(msg.sender, out);
    out << ", ";
  }

  // member: sequence
  {
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EndEffectorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: command
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << "\n";
  }

  // member: args
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "args: ";
    rosidl_generator_traits::value_to_yaml(msg.args, out);
    out << "\n";
  }

  // member: sender
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sender: ";
    rosidl_generator_traits::value_to_yaml(msg.sender, out);
    out << "\n";
  }

  // member: sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.sequence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EndEffectorCommand & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::EndEffectorCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::EndEffectorCommand & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::EndEffectorCommand>()
{
  return "baxter_core_msgs::msg::EndEffectorCommand";
}

template<>
inline const char * name<baxter_core_msgs::msg::EndEffectorCommand>()
{
  return "baxter_core_msgs/msg/EndEffectorCommand";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::EndEffectorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::EndEffectorCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::EndEffectorCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__TRAITS_HPP_
