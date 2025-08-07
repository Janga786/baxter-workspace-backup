// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:srv/BridgePublishersAuth.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__TRAITS_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/srv/detail/bridge_publishers_auth__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const BridgePublishersAuth_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: topic
  {
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << ", ";
  }

  // member: user
  {
    out << "user: ";
    rosidl_generator_traits::value_to_yaml(msg.user, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BridgePublishersAuth_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: topic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "topic: ";
    rosidl_generator_traits::value_to_yaml(msg.topic, out);
    out << "\n";
  }

  // member: user
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "user: ";
    rosidl_generator_traits::value_to_yaml(msg.user, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BridgePublishersAuth_Request & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::srv::BridgePublishersAuth_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::BridgePublishersAuth_Request & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::BridgePublishersAuth_Request>()
{
  return "baxter_core_msgs::srv::BridgePublishersAuth_Request";
}

template<>
inline const char * name<baxter_core_msgs::srv::BridgePublishersAuth_Request>()
{
  return "baxter_core_msgs/srv/BridgePublishersAuth_Request";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::BridgePublishersAuth_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::BridgePublishersAuth_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::srv::BridgePublishersAuth_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'publishers'
#include "baxter_core_msgs/msg/detail/bridge_publisher__traits.hpp"

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const BridgePublishersAuth_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: publishers
  {
    if (msg.publishers.size() == 0) {
      out << "publishers: []";
    } else {
      out << "publishers: [";
      size_t pending_items = msg.publishers.size();
      for (auto item : msg.publishers) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: forced_left
  {
    out << "forced_left: ";
    rosidl_generator_traits::value_to_yaml(msg.forced_left, out);
    out << ", ";
  }

  // member: forced_right
  {
    out << "forced_right: ";
    rosidl_generator_traits::value_to_yaml(msg.forced_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BridgePublishersAuth_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: publishers
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.publishers.size() == 0) {
      out << "publishers: []\n";
    } else {
      out << "publishers:\n";
      for (auto item : msg.publishers) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: forced_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forced_left: ";
    rosidl_generator_traits::value_to_yaml(msg.forced_left, out);
    out << "\n";
  }

  // member: forced_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forced_right: ";
    rosidl_generator_traits::value_to_yaml(msg.forced_right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BridgePublishersAuth_Response & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::srv::BridgePublishersAuth_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::BridgePublishersAuth_Response & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::BridgePublishersAuth_Response>()
{
  return "baxter_core_msgs::srv::BridgePublishersAuth_Response";
}

template<>
inline const char * name<baxter_core_msgs::srv::BridgePublishersAuth_Response>()
{
  return "baxter_core_msgs/srv/BridgePublishersAuth_Response";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::BridgePublishersAuth_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::BridgePublishersAuth_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::srv::BridgePublishersAuth_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<baxter_core_msgs::srv::BridgePublishersAuth>()
{
  return "baxter_core_msgs::srv::BridgePublishersAuth";
}

template<>
inline const char * name<baxter_core_msgs::srv::BridgePublishersAuth>()
{
  return "baxter_core_msgs/srv/BridgePublishersAuth";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::BridgePublishersAuth>
  : std::integral_constant<
    bool,
    has_fixed_size<baxter_core_msgs::srv::BridgePublishersAuth_Request>::value &&
    has_fixed_size<baxter_core_msgs::srv::BridgePublishersAuth_Response>::value
  >
{
};

template<>
struct has_bounded_size<baxter_core_msgs::srv::BridgePublishersAuth>
  : std::integral_constant<
    bool,
    has_bounded_size<baxter_core_msgs::srv::BridgePublishersAuth_Request>::value &&
    has_bounded_size<baxter_core_msgs::srv::BridgePublishersAuth_Response>::value
  >
{
};

template<>
struct is_service<baxter_core_msgs::srv::BridgePublishersAuth>
  : std::true_type
{
};

template<>
struct is_service_request<baxter_core_msgs::srv::BridgePublishersAuth_Request>
  : std::true_type
{
};

template<>
struct is_service_response<baxter_core_msgs::srv::BridgePublishersAuth_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__BRIDGE_PUBLISHERS_AUTH__TRAITS_HPP_
