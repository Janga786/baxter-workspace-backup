// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from baxter_core_msgs:msg/EndEffectorCommand.idl
// generated code does not contain a copyright notice

#ifndef BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__BUILDER_HPP_
#define BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "baxter_core_msgs/msg/detail/end_effector_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace baxter_core_msgs
{

namespace msg
{

namespace builder
{

class Init_EndEffectorCommand_sequence
{
public:
  explicit Init_EndEffectorCommand_sequence(::baxter_core_msgs::msg::EndEffectorCommand & msg)
  : msg_(msg)
  {}
  ::baxter_core_msgs::msg::EndEffectorCommand sequence(::baxter_core_msgs::msg::EndEffectorCommand::_sequence_type arg)
  {
    msg_.sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorCommand msg_;
};

class Init_EndEffectorCommand_sender
{
public:
  explicit Init_EndEffectorCommand_sender(::baxter_core_msgs::msg::EndEffectorCommand & msg)
  : msg_(msg)
  {}
  Init_EndEffectorCommand_sequence sender(::baxter_core_msgs::msg::EndEffectorCommand::_sender_type arg)
  {
    msg_.sender = std::move(arg);
    return Init_EndEffectorCommand_sequence(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorCommand msg_;
};

class Init_EndEffectorCommand_args
{
public:
  explicit Init_EndEffectorCommand_args(::baxter_core_msgs::msg::EndEffectorCommand & msg)
  : msg_(msg)
  {}
  Init_EndEffectorCommand_sender args(::baxter_core_msgs::msg::EndEffectorCommand::_args_type arg)
  {
    msg_.args = std::move(arg);
    return Init_EndEffectorCommand_sender(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorCommand msg_;
};

class Init_EndEffectorCommand_command
{
public:
  explicit Init_EndEffectorCommand_command(::baxter_core_msgs::msg::EndEffectorCommand & msg)
  : msg_(msg)
  {}
  Init_EndEffectorCommand_args command(::baxter_core_msgs::msg::EndEffectorCommand::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_EndEffectorCommand_args(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorCommand msg_;
};

class Init_EndEffectorCommand_id
{
public:
  Init_EndEffectorCommand_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EndEffectorCommand_command id(::baxter_core_msgs::msg::EndEffectorCommand::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_EndEffectorCommand_command(msg_);
  }

private:
  ::baxter_core_msgs::msg::EndEffectorCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::baxter_core_msgs::msg::EndEffectorCommand>()
{
  return baxter_core_msgs::msg::builder::Init_EndEffectorCommand_id();
}

}  // namespace baxter_core_msgs

#endif  // BAXTER_CORE_MSGS__MSG__DETAIL__END_EFFECTOR_COMMAND__BUILDER_HPP_
