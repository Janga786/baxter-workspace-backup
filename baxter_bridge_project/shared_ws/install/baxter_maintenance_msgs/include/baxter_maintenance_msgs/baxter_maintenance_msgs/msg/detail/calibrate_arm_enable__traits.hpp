// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmEnable.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__TRAITS_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_enable__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'data'
#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_data__traits.hpp"

namespace baxter_maintenance_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CalibrateArmEnable & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_enabled
  {
    out << "is_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.is_enabled, out);
    out << ", ";
  }

  // member: uid
  {
    out << "uid: ";
    rosidl_generator_traits::value_to_yaml(msg.uid, out);
    out << ", ";
  }

  // member: data
  {
    out << "data: ";
    to_flow_style_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateArmEnable & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: is_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.is_enabled, out);
    out << "\n";
  }

  // member: uid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uid: ";
    rosidl_generator_traits::value_to_yaml(msg.uid, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data:\n";
    to_block_style_yaml(msg.data, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateArmEnable & msg, bool use_flow_style = false)
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
  const baxter_maintenance_msgs::msg::CalibrateArmEnable & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_maintenance_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_maintenance_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_maintenance_msgs::msg::CalibrateArmEnable & msg)
{
  return baxter_maintenance_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_maintenance_msgs::msg::CalibrateArmEnable>()
{
  return "baxter_maintenance_msgs::msg::CalibrateArmEnable";
}

template<>
inline const char * name<baxter_maintenance_msgs::msg::CalibrateArmEnable>()
{
  return "baxter_maintenance_msgs/msg/CalibrateArmEnable";
}

template<>
struct has_fixed_size<baxter_maintenance_msgs::msg::CalibrateArmEnable>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_maintenance_msgs::msg::CalibrateArmEnable>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_maintenance_msgs::msg::CalibrateArmEnable>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_ENABLE__TRAITS_HPP_
