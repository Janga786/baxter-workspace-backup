// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateSources.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__BUILDER_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_maintenance_msgs/msg/detail/update_sources__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_maintenance_msgs
{

namespace msg
{

namespace builder
{

class Init_UpdateSources_sources
{
public:
  explicit Init_UpdateSources_sources(::baxter_maintenance_msgs::msg::UpdateSources & msg)
  : msg_(msg)
  {}
  ::baxter_maintenance_msgs::msg::UpdateSources sources(::baxter_maintenance_msgs::msg::UpdateSources::_sources_type arg)
  {
    msg_.sources = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateSources msg_;
};

class Init_UpdateSources_uuid
{
public:
  Init_UpdateSources_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateSources_sources uuid(::baxter_maintenance_msgs::msg::UpdateSources::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_UpdateSources_sources(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateSources msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_maintenance_msgs::msg::UpdateSources>()
{
  return baxter_maintenance_msgs::msg::builder::Init_UpdateSources_uuid();
}

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCES__BUILDER_HPP_
