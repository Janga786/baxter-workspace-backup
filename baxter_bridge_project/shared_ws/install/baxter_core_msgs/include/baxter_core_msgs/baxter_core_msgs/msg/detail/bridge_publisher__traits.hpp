// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/BridgePublisher.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/bridge_publisher__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BridgePublisher & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic
  {
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << ", ";
  }

  // member: user
  {
    out << "user: ";
    rosidl_generator_traits::value_to_yaml(msg.user, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BridgePublisher & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << "\n";
  }

  // member: user
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "user: ";
    rosidl_generator_traits::value_to_yaml(msg.user, out);
    out << "\n";
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BridgePublisher & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::BridgePublisher & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::BridgePublisher & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::BridgePublisher>()
{
  return "baxter_core_msgs::msg::BridgePublisher";
}

template<>
inline const char * name<baxter_core_msgs::msg::BridgePublisher>()
{
  return "baxter_core_msgs/msg/BridgePublisher";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::BridgePublisher>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::BridgePublisher>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::BridgePublisher>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__BRIDGE_PUBLISHER__TRAITS_HPP_
