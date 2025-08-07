// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/SEAJointState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/sea_joint_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const SEAJointState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: name
  {
    if (msg.name.size() == 0) {
      out << "name: []";
    } else {
      out << "name: [";
      size_t pending_items = msg.name.size();
      for (auto item : msg.name) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: commanded_position
  {
    if (msg.commanded_position.size() == 0) {
      out << "commanded_position: []";
    } else {
      out << "commanded_position: [";
      size_t pending_items = msg.commanded_position.size();
      for (auto item : msg.commanded_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: commanded_velocity
  {
    if (msg.commanded_velocity.size() == 0) {
      out << "commanded_velocity: []";
    } else {
      out << "commanded_velocity: [";
      size_t pending_items = msg.commanded_velocity.size();
      for (auto item : msg.commanded_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: commanded_acceleration
  {
    if (msg.commanded_acceleration.size() == 0) {
      out << "commanded_acceleration: []";
    } else {
      out << "commanded_acceleration: [";
      size_t pending_items = msg.commanded_acceleration.size();
      for (auto item : msg.commanded_acceleration) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: commanded_effort
  {
    if (msg.commanded_effort.size() == 0) {
      out << "commanded_effort: []";
    } else {
      out << "commanded_effort: [";
      size_t pending_items = msg.commanded_effort.size();
      for (auto item : msg.commanded_effort) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: actual_position
  {
    if (msg.actual_position.size() == 0) {
      out << "actual_position: []";
    } else {
      out << "actual_position: [";
      size_t pending_items = msg.actual_position.size();
      for (auto item : msg.actual_position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: actual_velocity
  {
    if (msg.actual_velocity.size() == 0) {
      out << "actual_velocity: []";
    } else {
      out << "actual_velocity: [";
      size_t pending_items = msg.actual_velocity.size();
      for (auto item : msg.actual_velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: actual_effort
  {
    if (msg.actual_effort.size() == 0) {
      out << "actual_effort: []";
    } else {
      out << "actual_effort: [";
      size_t pending_items = msg.actual_effort.size();
      for (auto item : msg.actual_effort) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gravity_model_effort
  {
    if (msg.gravity_model_effort.size() == 0) {
      out << "gravity_model_effort: []";
    } else {
      out << "gravity_model_effort: [";
      size_t pending_items = msg.gravity_model_effort.size();
      for (auto item : msg.gravity_model_effort) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gravity_only
  {
    if (msg.gravity_only.size() == 0) {
      out << "gravity_only: []";
    } else {
      out << "gravity_only: [";
      size_t pending_items = msg.gravity_only.size();
      for (auto item : msg.gravity_only) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: hysteresis_model_effort
  {
    if (msg.hysteresis_model_effort.size() == 0) {
      out << "hysteresis_model_effort: []";
    } else {
      out << "hysteresis_model_effort: [";
      size_t pending_items = msg.hysteresis_model_effort.size();
      for (auto item : msg.hysteresis_model_effort) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: crosstalk_model_effort
  {
    if (msg.crosstalk_model_effort.size() == 0) {
      out << "crosstalk_model_effort: []";
    } else {
      out << "crosstalk_model_effort: [";
      size_t pending_items = msg.crosstalk_model_effort.size();
      for (auto item : msg.crosstalk_model_effort) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: hyst_state
  {
    out << "hyst_state: ";
    rosidl_generator_traits::value_to_yaml(msg.hyst_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SEAJointState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.name.size() == 0) {
      out << "name: []\n";
    } else {
      out << "name:\n";
      for (auto item : msg.name) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: commanded_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commanded_position.size() == 0) {
      out << "commanded_position: []\n";
    } else {
      out << "commanded_position:\n";
      for (auto item : msg.commanded_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: commanded_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commanded_velocity.size() == 0) {
      out << "commanded_velocity: []\n";
    } else {
      out << "commanded_velocity:\n";
      for (auto item : msg.commanded_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: commanded_acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commanded_acceleration.size() == 0) {
      out << "commanded_acceleration: []\n";
    } else {
      out << "commanded_acceleration:\n";
      for (auto item : msg.commanded_acceleration) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: commanded_effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.commanded_effort.size() == 0) {
      out << "commanded_effort: []\n";
    } else {
      out << "commanded_effort:\n";
      for (auto item : msg.commanded_effort) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: actual_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.actual_position.size() == 0) {
      out << "actual_position: []\n";
    } else {
      out << "actual_position:\n";
      for (auto item : msg.actual_position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: actual_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.actual_velocity.size() == 0) {
      out << "actual_velocity: []\n";
    } else {
      out << "actual_velocity:\n";
      for (auto item : msg.actual_velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: actual_effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.actual_effort.size() == 0) {
      out << "actual_effort: []\n";
    } else {
      out << "actual_effort:\n";
      for (auto item : msg.actual_effort) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gravity_model_effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gravity_model_effort.size() == 0) {
      out << "gravity_model_effort: []\n";
    } else {
      out << "gravity_model_effort:\n";
      for (auto item : msg.gravity_model_effort) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gravity_only
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gravity_only.size() == 0) {
      out << "gravity_only: []\n";
    } else {
      out << "gravity_only:\n";
      for (auto item : msg.gravity_only) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: hysteresis_model_effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.hysteresis_model_effort.size() == 0) {
      out << "hysteresis_model_effort: []\n";
    } else {
      out << "hysteresis_model_effort:\n";
      for (auto item : msg.hysteresis_model_effort) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: crosstalk_model_effort
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.crosstalk_model_effort.size() == 0) {
      out << "crosstalk_model_effort: []\n";
    } else {
      out << "crosstalk_model_effort:\n";
      for (auto item : msg.crosstalk_model_effort) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: hyst_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hyst_state: ";
    rosidl_generator_traits::value_to_yaml(msg.hyst_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SEAJointState & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::SEAJointState & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::SEAJointState & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::SEAJointState>()
{
  return "baxter_core_msgs::msg::SEAJointState";
}

template<>
inline const char * name<baxter_core_msgs::msg::SEAJointState>()
{
  return "baxter_core_msgs/msg/SEAJointState";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::SEAJointState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::SEAJointState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::SEAJointState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__SEA_JOINT_STATE__TRAITS_HPP_
