// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/AnalogOutputCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/analog_output_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AnalogOutputCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AnalogOutputCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AnalogOutputCommand & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::AnalogOutputCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::AnalogOutputCommand & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::AnalogOutputCommand>()
{
  return "baxter_core_msgs::msg::AnalogOutputCommand";
}

template<>
inline const char * name<baxter_core_msgs::msg::AnalogOutputCommand>()
{
  return "baxter_core_msgs/msg/AnalogOutputCommand";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::AnalogOutputCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::AnalogOutputCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::AnalogOutputCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ANALOG_OUTPUT_COMMAND__TRAITS_HPP_
