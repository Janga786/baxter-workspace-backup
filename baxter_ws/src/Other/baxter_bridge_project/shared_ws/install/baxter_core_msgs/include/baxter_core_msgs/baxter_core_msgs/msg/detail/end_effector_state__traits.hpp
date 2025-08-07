// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/end_effector_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EndEffectorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    to_flow_style_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: enabled
  {
    out << "enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.enabled, out);
    out << ", ";
  }

  // member: calibrated
  {
    out << "calibrated: ";
    rosidl_generator_traits::value_to_yaml(msg.calibrated, out);
    out << ", ";
  }

  // member: ready
  {
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << ", ";
  }

  // member: moving
  {
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << ", ";
  }

  // member: gripping
  {
    out << "gripping: ";
    rosidl_generator_traits::value_to_yaml(msg.gripping, out);
    out << ", ";
  }

  // member: missed
  {
    out << "missed: ";
    rosidl_generator_traits::value_to_yaml(msg.missed, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << ", ";
  }

  // member: reverse
  {
    out << "reverse: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << ", ";
  }

  // member: force
  {
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << ", ";
  }

  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: command
  {
    out << "command: ";
    rosidl_generator_traits::value_to_yaml(msg.command, out);
    out << ", ";
  }

  // member: command_sender
  {
    out << "command_sender: ";
    rosidl_generator_traits::value_to_yaml(msg.command_sender, out);
    out << ", ";
  }

  // member: command_sequence
  {
    out << "command_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.command_sequence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EndEffectorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp:\n";
    to_block_style_yaml(msg.timestamp, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
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

  // member: calibrated
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "calibrated: ";
    rosidl_generator_traits::value_to_yaml(msg.calibrated, out);
    out << "\n";
  }

  // member: ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ready: ";
    rosidl_generator_traits::value_to_yaml(msg.ready, out);
    out << "\n";
  }

  // member: moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << "\n";
  }

  // member: gripping
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripping: ";
    rosidl_generator_traits::value_to_yaml(msg.gripping, out);
    out << "\n";
  }

  // member: missed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "missed: ";
    rosidl_generator_traits::value_to_yaml(msg.missed, out);
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

  // member: reverse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reverse: ";
    rosidl_generator_traits::value_to_yaml(msg.reverse, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position: ";
    rosidl_generator_traits::value_to_yaml(msg.position, out);
    out << "\n";
  }

  // member: force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "force: ";
    rosidl_generator_traits::value_to_yaml(msg.force, out);
    out << "\n";
  }

  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
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

  // member: command_sender
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_sender: ";
    rosidl_generator_traits::value_to_yaml(msg.command_sender, out);
    out << "\n";
  }

  // member: command_sequence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_sequence: ";
    rosidl_generator_traits::value_to_yaml(msg.command_sequence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EndEffectorState & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::EndEffectorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::EndEffectorState & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::EndEffectorState>()
{
  return "baxter_core_msgs::msg::EndEffectorState";
}

template<>
inline const char * name<baxter_core_msgs::msg::EndEffectorState>()
{
  return "baxter_core_msgs/msg/EndEffectorState";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::EndEffectorState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::EndEffectorState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::EndEffectorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_STATE__TRAITS_HPP_
