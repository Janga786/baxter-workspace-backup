// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_maintenance_msgs:msg/CalibrateArmData.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__TRAITS_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_maintenance_msgs/msg/detail/calibrate_arm_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_maintenance_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CalibrateArmData & msg,
  std::ostream & out)
{
  out << "{";
  // member: suppress_write_to_file
  {
    out << "suppress_write_to_file: ";
    rosidl_generator_traits::value_to_yaml(msg.suppress_write_to_file, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateArmData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: suppress_write_to_file
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "suppress_write_to_file: ";
    rosidl_generator_traits::value_to_yaml(msg.suppress_write_to_file, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateArmData & msg, bool use_flow_style = false)
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
  const baxter_maintenance_msgs::msg::CalibrateArmData & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_maintenance_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_maintenance_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_maintenance_msgs::msg::CalibrateArmData & msg)
{
  return baxter_maintenance_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_maintenance_msgs::msg::CalibrateArmData>()
{
  return "baxter_maintenance_msgs::msg::CalibrateArmData";
}

template<>
inline const char * name<baxter_maintenance_msgs::msg::CalibrateArmData>()
{
  return "baxter_maintenance_msgs/msg/CalibrateArmData";
}

template<>
struct has_fixed_size<baxter_maintenance_msgs::msg::CalibrateArmData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<baxter_maintenance_msgs::msg::CalibrateArmData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<baxter_maintenance_msgs::msg::CalibrateArmData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__CALIBRATE_ARM_DATA__TRAITS_HPP_
