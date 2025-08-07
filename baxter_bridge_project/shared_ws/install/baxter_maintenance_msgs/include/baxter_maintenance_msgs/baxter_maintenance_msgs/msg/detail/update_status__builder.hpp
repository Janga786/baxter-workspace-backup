// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_maintenance_msgs:msg/UpdateStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__BUILDER_HPP_
#define BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_maintenance_msgs/msg/detail/update_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_maintenance_msgs
{

namespace msg
{

namespace builder
{

class Init_UpdateStatus_long_description
{
public:
  explicit Init_UpdateStatus_long_description(::baxter_maintenance_msgs::msg::UpdateStatus & msg)
  : msg_(msg)
  {}
  ::baxter_maintenance_msgs::msg::UpdateStatus long_description(::baxter_maintenance_msgs::msg::UpdateStatus::_long_description_type arg)
  {
    msg_.long_description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateStatus msg_;
};

class Init_UpdateStatus_progress
{
public:
  explicit Init_UpdateStatus_progress(::baxter_maintenance_msgs::msg::UpdateStatus & msg)
  : msg_(msg)
  {}
  Init_UpdateStatus_long_description progress(::baxter_maintenance_msgs::msg::UpdateStatus::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return Init_UpdateStatus_long_description(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateStatus msg_;
};

class Init_UpdateStatus_status
{
public:
  Init_UpdateStatus_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateStatus_progress status(::baxter_maintenance_msgs::msg::UpdateStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_UpdateStatus_progress(msg_);
  }

private:
  ::baxter_maintenance_msgs::msg::UpdateStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_maintenance_msgs::msg::UpdateStatus>()
{
  return baxter_maintenance_msgs::msg::builder::Init_UpdateStatus_status();
}

}  // namespace baxter_maintenance_msgs

#endif  // BAXTER_MAINTENANCE_MSGS__MSG__DETAIL__UPDATE_STATUS__BUILDER_HPP_
