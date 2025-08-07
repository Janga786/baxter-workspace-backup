// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/HeadPanCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/head_pan_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_HeadPanCommand_enable_pan_request
{
public:
  explicit Init_HeadPanCommand_enable_pan_request(::baxter_core_msgs::msg::HeadPanCommand & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::HeadPanCommand enable_pan_request(::baxter_core_msgs::msg::HeadPanCommand::_enable_pan_request_type arg)
  {
    msg_.enable_pan_request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::HeadPanCommand msg_;
};

class Init_HeadPanCommand_speed_ratio
{
public:
  explicit Init_HeadPanCommand_speed_ratio(::baxter_core_msgs::msg::HeadPanCommand & msg)
  : msg_(msg)
  {}
  Init_HeadPanCommand_enable_pan_request speed_ratio(::baxter_core_msgs::msg::HeadPanCommand::_speed_ratio_type arg)
  {
    msg_.speed_ratio = std::move(arg);
    return Init_HeadPanCommand_enable_pan_request(msg_);
  }

private:
  ::baxter_core_msgs::msg::HeadPanCommand msg_;
};

class Init_HeadPanCommand_target
{
public:
  Init_HeadPanCommand_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeadPanCommand_speed_ratio target(::baxter_core_msgs::msg::HeadPanCommand::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_HeadPanCommand_speed_ratio(msg_);
  }

private:
  ::baxter_core_msgs::msg::HeadPanCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::HeadPanCommand>()
{
  return baxter_core_msgs::msg::builder::Init_HeadPanCommand_target();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__HEAD_PAN_COMMAND__BUILDER_HPP_
