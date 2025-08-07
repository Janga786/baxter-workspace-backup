// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:msg/CollisionAvoidanceState.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__TRAITS_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/msg/detail/collision_avoidance_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace baxter_core_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const CollisionAvoidanceState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: other_arm
  {
    out << "other_arm: ";
    rosidl_generator_traits::value_to_yaml(msg.other_arm, out);
    out << ", ";
  }

  // member: collision_object
  {
    if (msg.collision_object.size() == 0) {
      out << "collision_object: []";
    } else {
      out << "collision_object: [";
      size_t pending_items = msg.collision_object.size();
      for (auto item : msg.collision_object) {
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
  const CollisionAvoidanceState & msg,
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

  // member: other_arm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "other_arm: ";
    rosidl_generator_traits::value_to_yaml(msg.other_arm, out);
    out << "\n";
  }

  // member: collision_object
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.collision_object.size() == 0) {
      out << "collision_object: []\n";
    } else {
      out << "collision_object:\n";
      for (auto item : msg.collision_object) {
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

inline std::string to_yaml(const CollisionAvoidanceState & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::msg::CollisionAvoidanceState & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::msg::CollisionAvoidanceState & msg)
{
  return baxter_core_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::msg::CollisionAvoidanceState>()
{
  return "baxter_core_msgs::msg::CollisionAvoidanceState";
}

template<>
inline const char * name<baxter_core_msgs::msg::CollisionAvoidanceState>()
{
  return "baxter_core_msgs/msg/CollisionAvoidanceState";
}

template<>
struct has_fixed_size<baxter_core_msgs::msg::CollisionAvoidanceState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::msg::CollisionAvoidanceState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::msg::CollisionAvoidanceState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__COLLISION_AVOIDANCE_STATE__TRAITS_HPP_
