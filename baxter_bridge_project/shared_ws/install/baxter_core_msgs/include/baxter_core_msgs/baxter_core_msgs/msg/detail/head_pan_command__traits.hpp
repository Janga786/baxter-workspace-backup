// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/HeadPanCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/head_pan_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HeadPanCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: target
  {
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
    out << ", ";
  }

  // member: speed_ratio
  {
    out << "speed_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_ratio, out);
    out << ", ";
  }

  // member: enable_pan_request
  {
    out << "enable_pan_request: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_pan_request, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HeadPanCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target: ";
    rosidl_generator_traits::value_to_yaml(msg.target, out);
    out << "\n";
  }

  // member: speed_ratio
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_ratio: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_ratio, out);
    out << "\n";
  }

  // member: enable_pan_request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable_pan_request: ";
    rosidl_generator_traits::value_to_yaml(msg.enable_pan_request, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HeadPanCommand & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::HeadPanCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::HeadPanCommand & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::HeadPanCommand>()
{
  return "baxter_core_msgs::msg::HeadPanCommand";
}

template<>
inline const char * name<baxter_core_msgs::msg::HeadPanCommand>()
{
  return "baxter_core_msgs/msg/HeadPanCommand";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::HeadPanCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::HeadPanCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<baxter_core_msgs::msg::HeadPanCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__TRAITS_HPP_
