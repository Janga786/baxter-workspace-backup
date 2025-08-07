// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateSources.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__TRAITS_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "baxter_maintenance_msgs/msg/detail/update_sources__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'sources'
#include "baxter_maintenance_msgs/msg/detail/update_source__traits.hpp"

namespace baxter_maintenance_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const UpdateSources & msg,
  std::ostream & out)
{
  out << "{";
  // member: uuid
  {
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << ", ";
  }

  // member: sources
  {
    if (msg.sources.size() == 0) {
      out << "sources: []";
    } else {
      out << "sources: [";
      size_t pending_items = msg.sources.size();
      for (auto item : msg.sources) {
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
  const UpdateSources & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: uuid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uuid: ";
    rosidl_generator_traits::value_to_yaml(msg.uuid, out);
    out << "\n";
  }

  // member: sources
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.sources.size() == 0) {
      out << "sources: []\n";
    } else {
      out << "sources:\n";
      for (auto item : msg.sources) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateSources & msg, bool use_flow_style = false)
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
  const baxter_maintenance_msgs::msg::UpdateSources & msg,
  std::ostream & out, size_t indentation = 0)
{
  baxter_maintenance_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use baxter_maintenance_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const baxter_maintenance_msgs::msg::UpdateSources & msg)
{
  return baxter_maintenance_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<baxter_maintenance_msgs::msg::UpdateSources>()
{
  return "baxter_maintenance_msgs::msg::UpdateSources";
}

template<>
inline const char * name<baxter_maintenance_msgs::msg::UpdateSources>()
{
  return "baxter_maintenance_msgs/msg/UpdateSources";
}

template<>
struct has_fixed_size<baxter_maintenance_msgs::msg::UpdateSources>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<baxter_maintenance_msgs::msg::UpdateSources>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<baxter_maintenance_msgs::msg::UpdateSources>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__TRAITS_HPP_
