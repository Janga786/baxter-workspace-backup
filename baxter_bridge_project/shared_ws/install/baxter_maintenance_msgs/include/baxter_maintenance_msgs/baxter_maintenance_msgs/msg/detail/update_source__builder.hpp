// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateSource.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__BUILDER_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_maintenance_msgs/msg/detail/update_source__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_maintenance_msgs
{

namespace msg
{

namespace builder
{

class Init_UpdateSource_uuid
{
public:
  explicit Init_UpdateSource_uuid(::baxter_maintenance_msgs::msg::UpdateSource & msg)
  : msg_(msg)
  {}
  ::baxter_maintenance_msgs::msg::UpdateSource uuid(::baxter_maintenance_msgs::msg::UpdateSource::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateSource msg_;
};

class Init_UpdateSource_version
{
public:
  explicit Init_UpdateSource_version(::baxter_maintenance_msgs::msg::UpdateSource & msg)
  : msg_(msg)
  {}
  Init_UpdateSource_uuid version(::baxter_maintenance_msgs::msg::UpdateSource::_version_type arg)
  {
    msg_.version = std::move(arg);
    return Init_UpdateSource_uuid(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateSource msg_;
};

class Init_UpdateSource_filename
{
public:
  explicit Init_UpdateSource_filename(::baxter_maintenance_msgs::msg::UpdateSource & msg)
  : msg_(msg)
  {}
  Init_UpdateSource_version filename(::baxter_maintenance_msgs::msg::UpdateSource::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return Init_UpdateSource_version(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateSource msg_;
};

class Init_UpdateSource_devname
{
public:
  Init_UpdateSource_devname()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateSource_filename devname(::baxter_maintenance_msgs::msg::UpdateSource::_devname_type arg)
  {
    msg_.devname = std::move(arg);
    return Init_UpdateSource_filename(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateSource msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_maintenance_msgs::msg::UpdateSource>()
{
  return baxter_maintenance_msgs::msg::builder::Init_UpdateSource_devname();
}

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_SOURCE__BUILDER_HPP_
