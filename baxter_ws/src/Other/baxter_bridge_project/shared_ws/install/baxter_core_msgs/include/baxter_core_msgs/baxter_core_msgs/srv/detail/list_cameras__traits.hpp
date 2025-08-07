// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_core_msgs:srv/ListCameras.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__TRAITS_HPP_
#define BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_core_msgs/srv/detail/list_cameras__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListCameras_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ListCameras_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ListCameras_Request & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::srv::ListCameras_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::ListCameras_Request & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::ListCameras_Request>()
{
  return "baxter_core_msgs::srv::ListCameras_Request";
}

template<>
inline const char * name<baxter_core_msgs::srv::ListCameras_Request>()
{
  return "baxter_core_msgs/srv/ListCameras_Request";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::ListCameras_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::ListCameras_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<baxter_core_msgs::srv::ListCameras_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace baxter_core_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const ListCameras_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: cameras
  {
    if (msg.cameras.size() == 0) {
      out << "cameras: []";
    } else {
      out << "cameras: [";
      size_t pending_items = msg.cameras.size();
      for (auto item : msg.cameras) {
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
  const ListCameras_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cameras
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cameras.size() == 0) {
      out << "cameras: []\n";
    } else {
      out << "cameras:\n";
      for (auto item : msg.cameras) {
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

inline std::string to_yaml(const ListCameras_Response & msg, bool use_flow_style = false)
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
  const baxter_core_msgs::srv::ListCameras_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_core_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_core_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const baxter_core_msgs::srv::ListCameras_Response & msg)
{
  return baxter_core_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_core_msgs::srv::ListCameras_Response>()
{
  return "baxter_core_msgs::srv::ListCameras_Response";
}

template<>
inline const char * name<baxter_core_msgs::srv::ListCameras_Response>()
{
  return "baxter_core_msgs/srv/ListCameras_Response";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::ListCameras_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_core_msgs::srv::ListCameras_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_core_msgs::srv::ListCameras_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<baxter_core_msgs::srv::ListCameras>()
{
  return "baxter_core_msgs::srv::ListCameras";
}

template<>
inline const char * name<baxter_core_msgs::srv::ListCameras>()
{
  return "baxter_core_msgs/srv/ListCameras";
}

template<>
struct has_fixed_size<baxter_core_msgs::srv::ListCameras>
  : std::integral_constant<
    bool,
    has_fixed_size<baxter_core_msgs::srv::ListCameras_Request>::value &&
    has_fixed_size<baxter_core_msgs::srv::ListCameras_Response>::value
  >
{
};

template<>
struct has_bounded_size<baxter_core_msgs::srv::ListCameras>
  : std::integral_constant<
    bool,
    has_bounded_size<baxter_core_msgs::srv::ListCameras_Request>::value &&
    has_bounded_size<baxter_core_msgs::srv::ListCameras_Response>::value
  >
{
};

template<>
struct is_service<baxter_core_msgs::srv::ListCameras>
  : std::true_type
{
};

template<>
struct is_service_request<baxter_core_msgs::srv::ListCameras_Request>
  : std::true_type
{
};

template<>
struct is_service_response<baxter_core_msgs::srv::ListCameras_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_CORE_MSGS__SRV__DETAIL__LIST_CAMERAS__TRAITS_HPP_
