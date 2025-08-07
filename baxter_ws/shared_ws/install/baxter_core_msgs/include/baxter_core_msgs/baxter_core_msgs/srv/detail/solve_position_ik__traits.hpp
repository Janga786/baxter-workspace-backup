// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:srv/SolvePositionIK.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__TRAITS_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/srv/detail/solve_position_ik__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose_stamp'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'seed_angles'
#include "sensor_msgs/msg/detail/joint_state__traits.hpp"

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SolvePositionIK_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose_stamp
  {
    if (msg.pose_stamp.size() == 0) {
      out << "pose_stamp: []";
    } else {
      out << "pose_stamp: [";
      size_t pending_items = msg.pose_stamp.size();
      for (auto item : msg.pose_stamp) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: seed_angles
  {
    if (msg.seed_angles.size() == 0) {
      out << "seed_angles: []";
    } else {
      out << "seed_angles: [";
      size_t pending_items = msg.seed_angles.size();
      for (auto item : msg.seed_angles) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: seed_mode
  {
    out << "seed_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.seed_mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SolvePositionIK_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pose_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose_stamp.size() == 0) {
      out << "pose_stamp: []\n";
    } else {
      out << "pose_stamp:\n";
      for (auto item : msg.pose_stamp) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: seed_angles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.seed_angles.size() == 0) {
      out << "seed_angles: []\n";
    } else {
      out << "seed_angles:\n";
      for (auto item : msg.seed_angles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: seed_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seed_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.seed_mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SolvePositionIK_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace baxter_core_msgs

namespace rosidl_generator_traits
{

[[deprecated("use baxter_core_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const baxter_core_msgs::srv::SolvePositionIK_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::SolvePositionIK_Request & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::SolvePositionIK_Request>()
{
  return "baxter_core_msgs::srv::SolvePositionIK_Request";
}

template<>
inline const char * name<baxter_core_msgs::srv::SolvePositionIK_Request>()
{
  return "baxter_core_msgs/srv/SolvePositionIK_Request";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::SolvePositionIK_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::SolvePositionIK_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::srv::SolvePositionIK_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'joints'
// already included above
// #include "sensor_msgs/msg/detail/joint_state__traits.hpp"

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SolvePositionIK_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: joints
  {
    if (msg.joints.size() == 0) {
      out << "joints: []";
    } else {
      out << "joints: [";
      size_t pending_items = msg.joints.size();
      for (auto item : msg.joints) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: is_valid
  {
    if (msg.is_valid.size() == 0) {
      out << "is_valid: []";
    } else {
      out << "is_valid: [";
      size_t pending_items = msg.is_valid.size();
      for (auto item : msg.is_valid) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: result_type
  {
    if (msg.result_type.size() == 0) {
      out << "result_type: []";
    } else {
      out << "result_type: [";
      size_t pending_items = msg.result_type.size();
      for (auto item : msg.result_type) {
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
  const SolvePositionIK_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints.size() == 0) {
      out << "joints: []\n";
    } else {
      out << "joints:\n";
      for (auto item : msg.joints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: is_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.is_valid.size() == 0) {
      out << "is_valid: []\n";
    } else {
      out << "is_valid:\n";
      for (auto item : msg.is_valid) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: result_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.result_type.size() == 0) {
      out << "result_type: []\n";
    } else {
      out << "result_type:\n";
      for (auto item : msg.result_type) {
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

inline std::string to_yaml(const SolvePositionIK_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace baxter_core_msgs

namespace rosidl_generator_traits
{

[[deprecated("use baxter_core_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const baxter_core_msgs::srv::SolvePositionIK_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::SolvePositionIK_Response & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::SolvePositionIK_Response>()
{
  return "baxter_core_msgs::srv::SolvePositionIK_Response";
}

template<>
inline const char * name<baxter_core_msgs::srv::SolvePositionIK_Response>()
{
  return "baxter_core_msgs/srv/SolvePositionIK_Response";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::SolvePositionIK_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::SolvePositionIK_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::srv::SolvePositionIK_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<baxter_core_msgs::srv::SolvePositionIK>()
{
  return "baxter_core_msgs::srv::SolvePositionIK";
}

template<>
inline const char * name<baxter_core_msgs::srv::SolvePositionIK>()
{
  return "baxter_core_msgs/srv/SolvePositionIK";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::SolvePositionIK>
  : std::integral_constant<
    bool,
    has_fixed_size<baxter_core_msgs::srv::SolvePositionIK_Request>::value &&
    has_fixed_size<baxter_core_msgs::srv::SolvePositionIK_Response>::value
  >
{
};

template<>
struct has_bounded_size<baxter_core_msgs::srv::SolvePositionIK>
  : std::integral_constant<
    bool,
    has_bounded_size<baxter_core_msgs::srv::SolvePositionIK_Request>::value &&
    has_bounded_size<baxter_core_msgs::srv::SolvePositionIK_Response>::value
  >
{
};

template<>
struct is_service<baxter_core_msgs::srv::SolvePositionIK>
  : std::true_type
{
};

template<>
struct is_service_request<baxter_core_msgs::srv::SolvePositionIK_Request>
  : std::true_type
{
};

template<>
struct is_service_response<baxter_core_msgs::srv::SolvePositionIK_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__SOLVE_POSITION_IK__TRAITS_HPP_
