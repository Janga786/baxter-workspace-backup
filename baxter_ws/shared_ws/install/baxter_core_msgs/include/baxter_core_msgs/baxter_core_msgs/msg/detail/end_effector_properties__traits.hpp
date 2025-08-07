// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorProperties.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/end_effector_properties__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EndEffectorProperties & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: ui_type
  {
    out << "ui_type: ";
    rosidl_generator_traits::value_to_yaml(msg.ui_type, out);
    out << ", ";
  }

  // member: manufacturer
  {
    out << "manufacturer: ";
    rosidl_generator_traits::value_to_yaml(msg.manufacturer, out);
    out << ", ";
  }

  // member: product
  {
    out << "product: ";
    rosidl_generator_traits::value_to_yaml(msg.product, out);
    out << ", ";
  }

  // member: serial_number
  {
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
    out << ", ";
  }

  // member: hardware_rev
  {
    out << "hardware_rev: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_rev, out);
    out << ", ";
  }

  // member: firmware_rev
  {
    out << "firmware_rev: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_rev, out);
    out << ", ";
  }

  // member: firmware_date
  {
    out << "firmware_date: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_date, out);
    out << ", ";
  }

  // member: has_calibration
  {
    out << "has_calibration: ";
    rosidl_generator_traits::value_to_yaml(msg.has_calibration, out);
    out << ", ";
  }

  // member: controls_grip
  {
    out << "controls_grip: ";
    rosidl_generator_traits::value_to_yaml(msg.controls_grip, out);
    out << ", ";
  }

  // member: senses_grip
  {
    out << "senses_grip: ";
    rosidl_generator_traits::value_to_yaml(msg.senses_grip, out);
    out << ", ";
  }

  // member: reverses_grip
  {
    out << "reverses_grip: ";
    rosidl_generator_traits::value_to_yaml(msg.reverses_grip, out);
    out << ", ";
  }

  // member: controls_force
  {
    out << "controls_force: ";
    rosidl_generator_traits::value_to_yaml(msg.controls_force, out);
    out << ", ";
  }

  // member: senses_force
  {
    out << "senses_force: ";
    rosidl_generator_traits::value_to_yaml(msg.senses_force, out);
    out << ", ";
  }

  // member: controls_position
  {
    out << "controls_position: ";
    rosidl_generator_traits::value_to_yaml(msg.controls_position, out);
    out << ", ";
  }

  // member: senses_position
  {
    out << "senses_position: ";
    rosidl_generator_traits::value_to_yaml(msg.senses_position, out);
    out << ", ";
  }

  // member: properties
  {
    out << "properties: ";
    rosidl_generator_traits::value_to_yaml(msg.properties, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EndEffectorProperties & msg,
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

  // member: ui_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ui_type: ";
    rosidl_generator_traits::value_to_yaml(msg.ui_type, out);
    out << "\n";
  }

  // member: manufacturer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "manufacturer: ";
    rosidl_generator_traits::value_to_yaml(msg.manufacturer, out);
    out << "\n";
  }

  // member: product
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "product: ";
    rosidl_generator_traits::value_to_yaml(msg.product, out);
    out << "\n";
  }

  // member: serial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_number: ";
    rosidl_generator_traits::value_to_yaml(msg.serial_number, out);
    out << "\n";
  }

  // member: hardware_rev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hardware_rev: ";
    rosidl_generator_traits::value_to_yaml(msg.hardware_rev, out);
    out << "\n";
  }

  // member: firmware_rev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firmware_rev: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_rev, out);
    out << "\n";
  }

  // member: firmware_date
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firmware_date: ";
    rosidl_generator_traits::value_to_yaml(msg.firmware_date, out);
    out << "\n";
  }

  // member: has_calibration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_calibration: ";
    rosidl_generator_traits::value_to_yaml(msg.has_calibration, out);
    out << "\n";
  }

  // member: controls_grip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controls_grip: ";
    rosidl_generator_traits::value_to_yaml(msg.controls_grip, out);
    out << "\n";
  }

  // member: senses_grip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "senses_grip: ";
    rosidl_generator_traits::value_to_yaml(msg.senses_grip, out);
    out << "\n";
  }

  // member: reverses_grip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reverses_grip: ";
    rosidl_generator_traits::value_to_yaml(msg.reverses_grip, out);
    out << "\n";
  }

  // member: controls_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controls_force: ";
    rosidl_generator_traits::value_to_yaml(msg.controls_force, out);
    out << "\n";
  }

  // member: senses_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "senses_force: ";
    rosidl_generator_traits::value_to_yaml(msg.senses_force, out);
    out << "\n";
  }

  // member: controls_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "controls_position: ";
    rosidl_generator_traits::value_to_yaml(msg.controls_position, out);
    out << "\n";
  }

  // member: senses_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "senses_position: ";
    rosidl_generator_traits::value_to_yaml(msg.senses_position, out);
    out << "\n";
  }

  // member: properties
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "properties: ";
    rosidl_generator_traits::value_to_yaml(msg.properties, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EndEffectorProperties & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::EndEffectorProperties & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::EndEffectorProperties & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::EndEffectorProperties>()
{
  return "baxter_core_msgs::msg::EndEffectorProperties";
}

template<>
inline const char * name<baxter_core_msgs::msg::EndEffectorProperties>()
{
  return "baxter_core_msgs/msg/EndEffectorProperties";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::EndEffectorProperties>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::EndEffectorProperties>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::EndEffectorProperties>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_PROPERTIES__TRAITS_HPP_
