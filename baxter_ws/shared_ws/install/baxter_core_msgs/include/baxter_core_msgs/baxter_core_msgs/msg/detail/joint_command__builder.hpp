// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/JointCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/joint_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_JointCommand_names
{
public:
  explicit Init_JointCommand_names(::baxter_core_msgs::msg::JointCommand & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::JointCommand names(::baxter_core_msgs::msg::JointCommand::_names_type arg)
  {
    msg_.names = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::JointCommand msg_;
};

class Init_JointCommand_command
{
public:
  explicit Init_JointCommand_command(::baxter_core_msgs::msg::JointCommand & msg)
  : msg_(msg)
  {}
  Init_JointCommand_names command(::baxter_core_msgs::msg::JointCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_JointCommand_names(msg_);
  }

private:
  ::baxter_core_msgs::msg::JointCommand msg_;
};

class Init_JointCommand_mode
{
public:
  Init_JointCommand_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointCommand_command mode(::baxter_core_msgs::msg::JointCommand::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_JointCommand_command(msg_);
  }

private:
  ::baxter_core_msgs::msg::JointCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::JointCommand>()
{
  return baxter_core_msgs::msg::builder::Init_JointCommand_mode();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__JOINT_COMMAND__BUILDER_HPP_
