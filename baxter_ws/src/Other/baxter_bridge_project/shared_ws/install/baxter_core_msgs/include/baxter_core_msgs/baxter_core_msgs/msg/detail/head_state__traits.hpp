// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/HeadState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/head_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HeadState & msg,
  std::ostream & out)
{
  out << "{";
  // member: pan
  {
    out << "pan: ";
    rosidl_generator_traits::value_to_yaml(msg.pan, out);
    out << ", ";
  }

  // member: is_turning
  {
    out << "is_turning: ";
    rosidl_generator_traits::value_to_yaml(msg.is_turning, out);
    out << ", ";
  }

  // member: is_nodding
  {
    out << "is_nodding: ";
    rosidl_generator_traits::value_to_yaml(msg.is_nodding, out);
    out << ", ";
  }

  // member: is_pan_enabled
  {
    out << "is_pan_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.is_pan_enabled, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HeadState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pan
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pan: ";
    rosidl_generator_traits::value_to_yaml(msg.pan, out);
    out << "\n";
  }

  // member: is_turning
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_turning: ";
    rosidl_generator_traits::value_to_yaml(msg.is_turning, out);
    out << "\n";
  }

  // member: is_nodding
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_nodding: ";
    rosidl_generator_traits::value_to_yaml(msg.is_nodding, out);
    out << "\n";
  }

  // member: is_pan_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_pan_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.is_pan_enabled, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HeadState & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::HeadState & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::HeadState & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::HeadState>()
{
  return "baxter_core_msgs::msg::HeadState";
}

template<>
inline const char * name<baxter_core_msgs::msg::HeadState>()
{
  return "baxter_core_msgs/msg/HeadState";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::HeadState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::HeadState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<baxter_core_msgs::msg::HeadState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_STATE__TRAITS_HPP_
