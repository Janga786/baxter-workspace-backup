// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/RobustControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/robust_controller_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_RobustControllerStatus_labels
{
public:
  explicit Init_RobustControllerStatus_labels(::baxter_core_msgs::msg::RobustControllerStatus & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::RobustControllerStatus labels(::baxter_core_msgs::msg::RobustControllerStatus::_labels_type arg)
  {
    msg_.labels = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::RobustControllerStatus msg_;
};

class Init_RobustControllerStatus_error_codes
{
public:
  explicit Init_RobustControllerStatus_error_codes(::baxter_core_msgs::msg::RobustControllerStatus & msg)
  : msg_(msg)
  {}
  Init_RobustControllerStatus_labels error_codes(::baxter_core_msgs::msg::RobustControllerStatus::_error_codes_type arg)
  {
    msg_.error_codes = std::move(arg);
    return Init_RobustControllerStatus_labels(msg_);
  }

private:
  ::baxter_core_msgs::msg::RobustControllerStatus msg_;
};

class Init_RobustControllerStatus_timed_out
{
public:
  explicit Init_RobustControllerStatus_timed_out(::baxter_core_msgs::msg::RobustControllerStatus & msg)
  : msg_(msg)
  {}
  Init_RobustControllerStatus_error_codes timed_out(::baxter_core_msgs::msg::RobustControllerStatus::_timed_out_type arg)
  {
    msg_.timed_out = std::move(arg);
    return Init_RobustControllerStatus_error_codes(msg_);
  }

private:
  ::baxter_core_msgs::msg::RobustControllerStatus msg_;
};

class Init_RobustControllerStatus_control_uid
{
public:
  explicit Init_RobustControllerStatus_control_uid(::baxter_core_msgs::msg::RobustControllerStatus & msg)
  : msg_(msg)
  {}
  Init_RobustControllerStatus_timed_out control_uid(::baxter_core_msgs::msg::RobustControllerStatus::_control_uid_type arg)
  {
    msg_.control_uid = std::move(arg);
    return Init_RobustControllerStatus_timed_out(msg_);
  }

private:
  ::baxter_core_msgs::msg::RobustControllerStatus msg_;
};

class Init_RobustControllerStatus_complete
{
public:
  explicit Init_RobustControllerStatus_complete(::baxter_core_msgs::msg::RobustControllerStatus & msg)
  : msg_(msg)
  {}
  Init_RobustControllerStatus_control_uid complete(::baxter_core_msgs::msg::RobustControllerStatus::_complete_type arg)
  {
    msg_.complete = std::move(arg);
    return Init_RobustControllerStatus_control_uid(msg_);
  }

private:
  ::baxter_core_msgs::msg::RobustControllerStatus msg_;
};

class Init_RobustControllerStatus_is_enabled
{
public:
  Init_RobustControllerStatus_is_enabled()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobustControllerStatus_complete is_enabled(::baxter_core_msgs::msg::RobustControllerStatus::_is_enabled_type arg)
  {
    msg_.is_enabled = std::move(arg);
    return Init_RobustControllerStatus_complete(msg_);
  }

private:
  ::baxter_core_msgs::msg::RobustControllerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::RobustControllerStatus>()
{
  return baxter_core_msgs::msg::builder::Init_RobustControllerStatus_is_enabled();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__ROBUST_CONTROLLER_STATUS__BUILDER_HPP_
