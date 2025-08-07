// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:srv/BridgePublishersForce.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__TRAITS_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/srv/detail/bridge_publishers_force__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const BridgePublishersForce_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_user
  {
    out << "left_user: ";
    rosidl_generator_traits::value_to_yaml(msg.left_user, out);
    out << ", ";
  }

  // member: right_user
  {
    out << "right_user: ";
    rosidl_generator_traits::value_to_yaml(msg.right_user, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BridgePublishersForce_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_user
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_user: ";
    rosidl_generator_traits::value_to_yaml(msg.left_user, out);
    out << "\n";
  }

  // member: right_user
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_user: ";
    rosidl_generator_traits::value_to_yaml(msg.right_user, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BridgePublishersForce_Request & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::srv::BridgePublishersForce_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::BridgePublishersForce_Request & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::BridgePublishersForce_Request>()
{
  return "baxter_core_msgs::srv::BridgePublishersForce_Request";
}

template<>
inline const char * name<baxter_core_msgs::srv::BridgePublishersForce_Request>()
{
  return "baxter_core_msgs/srv/BridgePublishersForce_Request";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::BridgePublishersForce_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::BridgePublishersForce_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::srv::BridgePublishersForce_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const BridgePublishersForce_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BridgePublishersForce_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BridgePublishersForce_Response & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::srv::BridgePublishersForce_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::BridgePublishersForce_Response & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::BridgePublishersForce_Response>()
{
  return "baxter_core_msgs::srv::BridgePublishersForce_Response";
}

template<>
inline const char * name<baxter_core_msgs::srv::BridgePublishersForce_Response>()
{
  return "baxter_core_msgs/srv/BridgePublishersForce_Response";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::BridgePublishersForce_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::BridgePublishersForce_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<baxter_core_msgs::srv::BridgePublishersForce_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<baxter_core_msgs::srv::BridgePublishersForce>()
{
  return "baxter_core_msgs::srv::BridgePublishersForce";
}

template<>
inline const char * name<baxter_core_msgs::srv::BridgePublishersForce>()
{
  return "baxter_core_msgs/srv/BridgePublishersForce";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::BridgePublishersForce>
  : std::integral_constant<
    bool,
    has_fixed_size<baxter_core_msgs::srv::BridgePublishersForce_Request>::value &&
    has_fixed_size<baxter_core_msgs::srv::BridgePublishersForce_Response>::value
  >
{
};

template<>
struct has_bounded_size<baxter_core_msgs::srv::BridgePublishersForce>
  : std::integral_constant<
    bool,
    has_bounded_size<baxter_core_msgs::srv::BridgePublishersForce_Request>::value &&
    has_bounded_size<baxter_core_msgs::srv::BridgePublishersForce_Response>::value
  >
{
};

template<>
struct is_service<baxter_core_msgs::srv::BridgePublishersForce>
  : std::true_type
{
};

template<>
struct is_service_request<baxter_core_msgs::srv::BridgePublishersForce_Request>
  : std::true_type
{
};

template<>
struct is_service_response<baxter_core_msgs::srv::BridgePublishersForce_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_FORCE__TRAITS_HPP_
