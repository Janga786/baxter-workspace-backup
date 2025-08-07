// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/CameraSettings.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/camera_settings__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'controls'
#include "baxter_core_msgs/msg/detail/camera_control__traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CameraSettings & msg,
  std::ostream & out)
{
  out << "{";
  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: fps
  {
    out << "fps: ";
    rosidl_generator_traits::value_to_yaml(msg.fps, out);
    out << ", ";
  }

  // member: controls
  {
    if (msg.controls.size() == 0) {
      out << "controls: []";
    } else {
      out << "controls: [";
      size_t pending_items = msg.controls.size();
      for (auto item : msg.controls) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraSettings & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: fps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fps: ";
    rosidl_generator_traits::value_to_yaml(msg.fps, out);
    out << "\n";
  }

  // member: controls
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.controls.size() == 0) {
      out << "controls: []\n";
    } else {
      out << "controls:\n";
      for (auto item : msg.controls) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraSettings & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::CameraSettings & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::CameraSettings & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::CameraSettings>()
{
  return "baxter_core_msgs::msg::CameraSettings";
}

template<>
inline const char * name<baxter_core_msgs::msg::CameraSettings>()
{
  return "baxter_core_msgs/msg/CameraSettings";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::CameraSettings>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::CameraSettings>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::CameraSettings>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__CAMERA_SETTINGS__TRAITS_HPP_
