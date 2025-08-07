// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/NavigatorState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/navigator_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavigatorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: button_names
  {
    if (msg.button_names.size() == 0) {
      out << "button_names: []";
    } else {
      out << "button_names: [";
      size_t pending_items = msg.button_names.size();
      for (auto item : msg.button_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: buttons
  {
    if (msg.buttons.size() == 0) {
      out << "buttons: []";
    } else {
      out << "buttons: [";
      size_t pending_items = msg.buttons.size();
      for (auto item : msg.buttons) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wheel
  {
    out << "wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel, out);
    out << ", ";
  }

  // member: light_names
  {
    if (msg.light_names.size() == 0) {
      out << "light_names: []";
    } else {
      out << "light_names: [";
      size_t pending_items = msg.light_names.size();
      for (auto item : msg.light_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: lights
  {
    if (msg.lights.size() == 0) {
      out << "lights: []";
    } else {
      out << "lights: [";
      size_t pending_items = msg.lights.size();
      for (auto item : msg.lights) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const NavigatorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: button_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.button_names.size() == 0) {
      out << "button_names: []\n";
    } else {
      out << "button_names:\n";
      for (auto item : msg.button_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: buttons
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.buttons.size() == 0) {
      out << "buttons: []\n";
    } else {
      out << "buttons:\n";
      for (auto item : msg.buttons) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wheel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wheel: ";
    rosidl_generator_traits::value_to_yaml(msg.wheel, out);
    out << "\n";
  }

  // member: light_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.light_names.size() == 0) {
      out << "light_names: []\n";
    } else {
      out << "light_names:\n";
      for (auto item : msg.light_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: lights
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.lights.size() == 0) {
      out << "lights: []\n";
    } else {
      out << "lights:\n";
      for (auto item : msg.lights) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigatorState & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::NavigatorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::NavigatorState & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::NavigatorState>()
{
  return "baxter_core_msgs::msg::NavigatorState";
}

template<>
inline const char * name<baxter_core_msgs::msg::NavigatorState>()
{
  return "baxter_core_msgs/msg/NavigatorState";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::NavigatorState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::NavigatorState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::NavigatorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__NAVIGATOR_STATE__TRAITS_HPP_
