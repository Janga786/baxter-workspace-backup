// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/RobustControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/robust_controller_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RobustControllerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: is_enabled
  {
    out << "is_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.is_enabled, out);
    out << ", ";
  }

  // member: complete
  {
    out << "complete: ";
    rosidl_generator_traits::value_to_yaml(msg.complete, out);
    out << ", ";
  }

  // member: control_uid
  {
    out << "control_uid: ";
    rosidl_generator_traits::value_to_yaml(msg.control_uid, out);
    out << ", ";
  }

  // member: timed_out
  {
    out << "timed_out: ";
    rosidl_generator_traits::value_to_yaml(msg.timed_out, out);
    out << ", ";
  }

  // member: error_codes
  {
    if (msg.error_codes.size() == 0) {
      out << "error_codes: []";
    } else {
      out << "error_codes: [";
      size_t pending_items = msg.error_codes.size();
      for (auto item : msg.error_codes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: labels
  {
    if (msg.labels.size() == 0) {
      out << "labels: []";
    } else {
      out << "labels: [";
      size_t pending_items = msg.labels.size();
      for (auto item : msg.labels) {
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
  const RobustControllerStatus & msg,
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

  // member: complete
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "complete: ";
    rosidl_generator_traits::value_to_yaml(msg.complete, out);
    out << "\n";
  }

  // member: control_uid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_uid: ";
    rosidl_generator_traits::value_to_yaml(msg.control_uid, out);
    out << "\n";
  }

  // member: timed_out
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timed_out: ";
    rosidl_generator_traits::value_to_yaml(msg.timed_out, out);
    out << "\n";
  }

  // member: error_codes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.error_codes.size() == 0) {
      out << "error_codes: []\n";
    } else {
      out << "error_codes:\n";
      for (auto item : msg.error_codes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: labels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.labels.size() == 0) {
      out << "labels: []\n";
    } else {
      out << "labels:\n";
      for (auto item : msg.labels) {
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

inline std::string to_yaml(const RobustControllerStatus & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::RobustControllerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::RobustControllerStatus & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::RobustControllerStatus>()
{
  return "baxter_core_msgs::msg::RobustControllerStatus";
}

template<>
inline const char * name<baxter_core_msgs::msg::RobustControllerStatus>()
{
  return "baxter_core_msgs/msg/RobustControllerStatus";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::RobustControllerStatus>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::RobustControllerStatus>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::RobustControllerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__TRAITS_HPP_
